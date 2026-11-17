#include "robocup_localization/ui/main_window.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "robocup_localization/app/user_config_path.hpp"

#include <QMessageBox>
#include <algorithm>
#include <array>
#include <cmath>
#include <QtGui>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numeric>
#include <vector>

#include "robot_config.h"
#include "field_coordinate/field_coordinate.hpp"

namespace
{

constexpr qreal kUiScale = 0.8;

int scaled(int value)
{
  return qRound(value * kUiScale);
}

void scaleMainWindowUi(QMainWindow& window, QWidget& central_widget)
{
  const auto widgets = central_widget.findChildren<QWidget*>(
      QString(), Qt::FindDirectChildrenOnly);

  for (QWidget* widget : widgets)
  {
    const QRect geometry = widget->geometry();
    widget->setGeometry(
        scaled(geometry.x()), scaled(geometry.y()),
        scaled(geometry.width()), scaled(geometry.height()));

    QFont font = widget->font();
    if (font.pointSizeF() > 0.0)
    {
      font.setPointSizeF(std::max(1.0, font.pointSizeF() * kUiScale));
      widget->setFont(font);
    }
  }

  window.setFixedSize(scaled(window.width()), scaled(window.height()));
}

void fitFieldScene(QGraphicsView& view, QGraphicsScene& scene)
{
  const QRectF map_rect(
      0.0, 0.0, FieldCoord::MAP_W, FieldCoord::MAP_H);
  scene.setSceneRect(map_rect);
  view.fitInView(map_rect, Qt::KeepAspectRatio);
}

bool mapToFieldScene(
    QGraphicsView& view,
    const QWidget& source,
    const QPoint& source_position,
    QPointF& scene_position)
{
  const QPoint viewport_position =
      view.viewport()->mapFrom(&source, source_position);
  if (!view.viewport()->rect().contains(viewport_position))
  {
    return false;
  }

  scene_position = view.mapToScene(viewport_position);
  scene_position.setX(std::clamp(
      scene_position.x(), 0.0, static_cast<double>(FieldCoord::MAP_W)));
  scene_position.setY(std::clamp(
      scene_position.y(), 0.0, static_cast<double>(FieldCoord::MAP_H)));
  return true;
}

}

MainWindow::MainWindow(
    robocup_localization::app::LocalizationSnapshotSource& snapshots,
    robocup_localization::app::LocalizationCommand& commands,
    QWidget* parent)
    : QMainWindow(parent),
      snapshots_(&snapshots),
      commands_(&commands),
      localization_snapshot_(snapshots.localizationSnapshot()),
      localization_config_(snapshots.localizationConfig()),
      ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  scaleMainWindowUi(*this, *ui->centralwidget);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  const auto& loop_rate = localization_config_.loop_rate;
  const auto interval_ms = [](double hz)
  {
    const double safe_hz = std::max(0.1, hz);
    return std::max(1, static_cast<int>(std::lround(1000.0 / safe_hz)));
  };

  m_DataTimer = new QTimer(this);
  connect(m_DataTimer, SIGNAL(timeout()), this, SLOT(main()));
  m_DataTimer->start(interval_ms(loop_rate.ui_data_hz));

  m_screenTimer = new QTimer(this);
  connect(m_screenTimer, SIGNAL(timeout()), this, SLOT(Print_Screen()));
  m_screenTimer->start(interval_ms(loop_rate.ui_render_hz));

  buf = QPixmap(FieldCoord::MAP_W, FieldCoord::MAP_H);
  buf.fill(QColor(34, 139, 34));

  {
    QPainter p(&buf);
    p.setRenderHint(QPainter::Antialiasing);

    int line_w = (int)(FieldCoord::LINE_W * FieldCoord::PX_PER_M_X);
    if (line_w < 1)
      line_w = 1;

    p.setPen(QPen(Qt::white, line_w));
    p.setBrush(Qt::NoBrush);

    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::B / 2),
               (int)(FieldCoord::A * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::B * FieldCoord::PX_PER_M_Y));

    p.drawLine(FieldCoord::toPixelX(0), FieldCoord::toPixelY(FieldCoord::B / 2),
               FieldCoord::toPixelX(0), FieldCoord::toPixelY(-FieldCoord::B / 2));

    int circle_r = (int)(FieldCoord::J / 2.0 * FieldCoord::PX_PER_M_X);
    p.drawEllipse(FieldCoord::toPixelX(0) - circle_r,
                  FieldCoord::toPixelY(0) - circle_r,
                  circle_r * 2, circle_r * 2);

    int goal_w = (int)(FieldCoord::C * FieldCoord::PX_PER_M_X);
    int goal_h = (int)(FieldCoord::D * FieldCoord::PX_PER_M_Y);
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2) - goal_w,
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);

    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M_Y));

    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::E),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M_Y));

    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M_Y));

    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::G),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M_Y));

    int mark_sz = (int)(FieldCoord::MARK_SZ / 2.0 * FieldCoord::PX_PER_M_X);
    if (mark_sz < 2)
      mark_sz = 2;

    p.setBrush(Qt::white);

    p.drawEllipse(FieldCoord::toPixelX(0) - mark_sz, FieldCoord::toPixelY(0) - mark_sz,
                  mark_sz * 2, mark_sz * 2);

    int pm0_x = FieldCoord::toPixelX(-FieldCoord::A / 2 + FieldCoord::I_DIST);
    int pm0_y = FieldCoord::toPixelY(0);
    p.drawEllipse(pm0_x - mark_sz, pm0_y - mark_sz, mark_sz * 2, mark_sz * 2);

    int pm1_x = FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::I_DIST);
    int pm1_y = FieldCoord::toPixelY(0);
    p.drawEllipse(pm1_x - mark_sz, pm1_y - mark_sz, mark_sz * 2, mark_sz * 2);

    p.setBrush(Qt::NoBrush);

    if (FieldCoord::L_ARC > 0.0)
    {
      int r = (int)(FieldCoord::L_ARC * FieldCoord::PX_PER_M_X);
      int x_l = FieldCoord::toPixelX(-FieldCoord::A / 2);
      int x_r = FieldCoord::toPixelX(FieldCoord::A / 2);
      int y_t = FieldCoord::toPixelY(FieldCoord::B / 2);
      int y_b = FieldCoord::toPixelY(-FieldCoord::B / 2);
      p.drawArc(x_l - r, y_t - r, 2 * r, 2 * r, 270 * 16, 90 * 16);
      p.drawArc(x_r - r, y_t - r, 2 * r, 2 * r, 180 * 16, 90 * 16);
      p.drawArc(x_l - r, y_b - r, 2 * r, 2 * r, 0 * 16, 90 * 16);
      p.drawArc(x_r - r, y_b - r, 2 * r, 2 * r, 90 * 16, 90 * 16);
    }

    p.end();
  }

  scene = new QGraphicsScene(this);
  commands_->previewRobotPosition(
      FieldCoord::MAP_W / 2.0f, FieldCoord::MAP_H / 2.0f);
}

void MainWindow::main()
{
  setting();
}

void MainWindow::Print_Screen_Ball()
{
  if (!scene)
  {
    return;
  }

  const BALL& ball = localization_snapshot_.world.ball;
  const bool ball_in_range =
      ball.x >= FieldCoord::F_X_MIN && ball.x <= FieldCoord::F_X_MAX &&
      ball.y >= FieldCoord::F_Y_MIN && ball.y <= FieldCoord::F_Y_MAX;

  if (!ball_in_range)
  {
    return;
  }

  QPen blackPen(Qt::black);
  QBrush redBrush(Qt::red);
  QBrush yellowBrush(Qt::yellow);
  QBrush blueBrush(Qt::blue);
  QBrush pinkBrush(QColor(255, 105, 180));

  if (manual_ball_editing_)
  {
    scene->addEllipse(
        cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5,
        11, 11, blackPen, blueBrush);
    return;
  }

  if (ball.from_udp)
  {
    scene->addEllipse(
        cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5,
        11, 11, blackPen, pinkBrush);
    return;
  }

  if (ball.noballcnt == 0)
  {
    scene->addEllipse(
        cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5,
        11, 11, blackPen, redBrush);

    const int prediction_level = localization_snapshot_.world.ball_prediction_level;
    scene->addLine(
        cvt_Print_xy(ball.x),
        cvt_Print_xy(ball.y),
        cvt_Print_xy(ball.x + ball.speed_x * prediction_level),
        cvt_Print_xy(ball.y + ball.speed_y * prediction_level),
        blackPen);
    return;
  }

  if (ball.x != 0 || ball.y != 0)
  {
    scene->addEllipse(
        cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5,
        11, 11, blackPen, yellowBrush);
  }
}

void MainWindow::setting()
{
  if (setting_flag == 0)
  {
    const auto current = snapshots_->localizationSnapshot().robot;
    float odom_forward = current.odom_forward;
    float odom_backward = current.odom_backward;
    float odom_left = current.odom_left;
    float odom_right = current.odom_right;

    const std::string rel = robocup_localization::app::odomParamRelativePath();
    const std::string param =
        rel.empty() ? std::string()
                    : robocup_localization::app::resourcePath(rel);

    ifstream input(param);
    if (input.is_open())
    {
      input >> particle_range;
      input >> odom_forward;
      input >> odom_backward;
      input >> odom_left;
      input >> odom_right;
      printf("[LocalNode] Odom Load: %s\n", param.c_str());
    }
    else
    {
      printf("[LocalNode] No Odom Data: %s\n",
             param.empty() ? "로봇 번호 미정의(robot_config.h)" : param.c_str());
    }

    ui->particle_range_value->setValue(particle_range);
    ui->odom_fx_value->setValue(odom_forward);
    ui->odom_bx_value->setValue(odom_backward);
    ui->odom_ly_value->setValue(odom_left);
    ui->odom_ry_value->setValue(odom_right);
    setting_flag = 1;
  }

  particle_range = ui->particle_range_value->value();
  commands_->setParticleRange(particle_range);
  commands_->setRobotOdometryCalibration(
      ui->odom_fx_value->value(), ui->odom_bx_value->value(),
      ui->odom_ly_value->value(), ui->odom_ry_value->value());
}

void MainWindow::applyOdometryInputState(bool metric)
{
  // Metric odometry ignores the coefficients, so the inputs are disabled and dimmed.
  QWidget *boxes[] = {ui->odom_fx_value, ui->odom_bx_value,
                      ui->odom_ly_value, ui->odom_ry_value};
  QWidget *labels[] = {ui->label_9, ui->label_13, ui->label_10, ui->label_14};

  if (odom_widget_styles_.isEmpty())
  {
    for (QWidget *w : boxes)
      odom_widget_styles_.insert(w, w->styleSheet());
    for (QWidget *w : labels)
      odom_widget_styles_.insert(w, w->styleSheet());
  }

  QString tooltip;
  QString box_style;
  QString label_style;
  if (metric)
  {
    tooltip = QString::fromUtf8(
        "odometry.input: metric\n"
        "ikcoordinate 를 실제 이동량(cm)으로 쓰므로 odom 계수는 쓰지 않는다.");
    box_style =
        "background-color: rgb(24, 24, 24);"
        "color: rgb(60, 60, 60);"
        "border: 1px solid rgb(40, 40, 40);";
    label_style = "color: rgb(70, 70, 70);";
  }

  for (QWidget *w : boxes)
  {
    w->setEnabled(!metric);
    w->setToolTip(tooltip);
    if (metric)
      w->setStyleSheet(box_style);
    else
      w->setStyleSheet(odom_widget_styles_.value(w));
  }
  for (QWidget *w : labels)
  {
    w->setToolTip(tooltip);
    if (metric)
      w->setStyleSheet(label_style);
    else
      w->setStyleSheet(odom_widget_styles_.value(w));
  }
}

void MainWindow::Print_Screen()
{
  localization_snapshot_ = snapshots_->localizationSnapshot();

  const int odom_input_state =
      static_cast<int>(localization_snapshot_.robot.odom_metric);
  if (odom_input_state != odom_input_ui_state_)
  {
    applyOdometryInputState(localization_snapshot_.robot.odom_metric);
    odom_input_ui_state_ = odom_input_state;
  }
  if (manual_ball_editing_ && manual_ball_placed_ &&
      !localization_snapshot_.world.manual_ball_active)
  {
    manual_ball_editing_ = false;
    manual_ball_placed_ = false;
  }

  if (scene)
  {
    delete scene;
  }

  char TEXT[256];
  sprintf(TEXT, "BALL_X : %d", (int)localization_snapshot_.world.ball.x);
  ui->BALL_X_value->setText(TEXT);
  sprintf(TEXT, "BALL_Y : %d", (int)localization_snapshot_.world.ball.y);
  ui->BALL_Y_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_X : %d", (int)localization_snapshot_.robot.pose.x);
  ui->ROBOT_X_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_Y : %d", (int)localization_snapshot_.robot.pose.y);
  ui->ROBOT_Y_value->setText(TEXT);
  sprintf(TEXT, "IK_X : %d", (int)localization_snapshot_.robot.ik_x);
  ui->IK_X_value->setText(TEXT);
  sprintf(TEXT, "IK_Y : %d", (int)localization_snapshot_.robot.ik_y);
  ui->IK_Y_value->setText(TEXT);

  scene = new QGraphicsScene(this);
  ui->screen->setScene(scene);
  scene->addPixmap(buf);
  fitFieldScene(*ui->screen, *scene);

  QBrush blueBrush(Qt::blue);
  QBrush redBrush(Qt::red);
  QBrush greenBrush(Qt::green);
  QBrush yellowBrush(Qt::yellow);
  QBrush blackBrush(Qt::black);
  QBrush grayBrush(Qt::gray);
  QPen grayPen(Qt::gray);
  grayPen.setWidth(1);
  QPen redPen(Qt::red);
  redPen.setWidth(2);
  QPen redPen3(Qt::red);
  redPen3.setWidth(3);
  QPen greenPen(Qt::green);
  greenPen.setWidth(2);
  QPen bluePen(Qt::blue);
  bluePen.setWidth(2);
  QPen blackPen1(Qt::black);
  blackPen1.setWidth(1);
  QPen blackPen(Qt::black);
  blackPen.setWidth(2);
  QPen blackPen3(Qt::black);
  blackPen3.setWidth(3);
  QPen LocalPen1(Qt::darkGreen);
  LocalPen1.setWidth(5);
  QPen LocalPen2(Qt::green);
  LocalPen2.setWidth(5);
  QPen LocalPen3(Qt::yellow);
  LocalPen3.setWidth(5);
  QPen LocalPen4(Qt::red);
  LocalPen4.setWidth(5);
  QPen yellowPen(Qt::yellow);
  yellowPen.setWidth(3);
  QPen targetPen(Qt::red);
  targetPen.setWidth(3);
  QPen targetlinePen(Qt::gray);
  targetlinePen.setWidth(2);

  for (const auto &obs : localization_snapshot_.world.obstacles)
  {
    if (obs.x != 0 && obs.y != 0)
    {
      scene->addRect(cvt_Print_xy(obs.x) - 5,
                     cvt_Print_xy(obs.y) - 5, 10, 10, blackPen,
                     blackBrush);
    }
  }

  for (const auto &obs : localization_snapshot_.world.manual_obstacles)
  {
    scene->addRect(cvt_Print_xy(obs.x) - 6,
                   cvt_Print_xy(obs.y) - 6, 12, 12, redPen,
                   redBrush);
  }

  for (const auto& particle : localization_snapshot_.particle_filter.particles)
  {
    scene->addLine(
        cvt_Print_xy(particle.x), cvt_Print_xy(particle.y),
        cvt_Print_xy(particle.x), cvt_Print_xy(particle.y), grayPen);
  }

  {
    // Last MCL cycle before resampling, sampled evenly across the weight ranking so the
    // whole spread shows; color spans min (blue) to max (red), heaviest drawn last.
    constexpr std::size_t SCORED_DRAW_COUNT = 300;
    const auto& scored = localization_snapshot_.particle_filter.scored_particles;
    const auto& weights = localization_snapshot_.particle_filter.scored_weights;
    if (!scored.empty() && scored.size() == weights.size())
    {
      std::vector<std::size_t> order(scored.size());
      std::iota(order.begin(), order.end(), 0);
      std::sort(order.begin(), order.end(),
                [&weights](std::size_t a, std::size_t b) { return weights[a] < weights[b]; });

      const double min_weight = weights[order.front()];
      const double range = weights[order.back()] - min_weight;
      const std::size_t count = std::min(SCORED_DRAW_COUNT, order.size());
      if (range > 0.0)
      {
        for (std::size_t k = 0; k < count; ++k)
        {
          std::size_t rank = order.size() - 1;
          if (count > 1)
          {
            rank = k * (order.size() - 1) / (count - 1);
          }
          const std::size_t i = order[rank];
          const double ratio = std::clamp((weights[i] - min_weight) / range, 0.0, 1.0);
          const QColor color = QColor::fromHsvF((1.0 - ratio) * 240.0 / 360.0, 1.0, 1.0);
          scene->addEllipse(
              cvt_Print_xy(scored[i].x) - 2, cvt_Print_xy(scored[i].y) - 2, 4, 4,
              QPen(color), QBrush(color));
        }
      }
    }
  }

  {
    const auto& inference = localization_snapshot_.inference;

    if (localization_config_.inference.reloc && inference.reloc.valid)
    {
      QPen cyanPen(QColor(0, 200, 200));
      cyanPen.setWidth(1);
      QBrush cyanBrush(QColor(0, 200, 200, 140));
      for (const auto& candidate : inference.reloc.candidates)
      {
        const double px = cvt_Print_xy(FieldCoord::toPixelX(candidate.x_m));
        const double py = cvt_Print_xy(FieldCoord::toPixelY(candidate.y_m));
        int r = 2 + static_cast<int>(candidate.prob * 10.0);
        if (r > 8)
          r = 8;
        scene->addEllipse(px - r, py - r, r * 2, r * 2, cyanPen, cyanBrush);
      }
    }

    if (localization_config_.inference.mlp && inference.mlp.valid)
    {
      QPen magentaPen(QColor(255, 0, 255));
      magentaPen.setWidth(3);
      QBrush magentaBrush(QColor(255, 0, 255, 90));
      const double px = cvt_Print_xy(FieldCoord::toPixelX(inference.mlp.x_m));
      const double py = cvt_Print_xy(FieldCoord::toPixelY(inference.mlp.y_m));
      const int r = 10;
      scene->addEllipse(px - r, py - r, r * 2, r * 2, magentaPen, magentaBrush);
      scene->addLine(px - r, py, px + r, py, magentaPen);
      scene->addLine(px, py - r, px, py + r, magentaPen);
    }
  }

  if (localization_snapshot_.robot.trail.size() > 0)
  {
    scene->addLine(
        cvt_Print_xy(localization_snapshot_.robot.pose.x), cvt_Print_xy(localization_snapshot_.robot.pose.y),
        cvt_Print_xy(
            localization_snapshot_.robot.trail[localization_snapshot_.robot.trail.size() - 1].X),
        cvt_Print_xy(
            localization_snapshot_.robot.trail[localization_snapshot_.robot.trail.size() - 1].Y),
        blackPen3);
    for (int i = 1; i < localization_snapshot_.robot.trail.size(); i++)
    {
      scene->addLine(cvt_Print_xy(localization_snapshot_.robot.trail[i - 1].X),
                     cvt_Print_xy(localization_snapshot_.robot.trail[i - 1].Y),
                     cvt_Print_xy(localization_snapshot_.robot.trail[i].X),
                     cvt_Print_xy(localization_snapshot_.robot.trail[i].Y), blackPen3);
    }
  }

  for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
  {
    if (!FieldCoord::ALL_POINTS[i].active)
      continue;
    if (localization_snapshot_.particle_filter.filter.Local_point_on_off[i] == 1)
    {
      scene->addEllipse(cvt_Print_xy(localization_snapshot_.particle_filter.filter.Local_point_x[i]) - 4,
                        cvt_Print_xy(localization_snapshot_.particle_filter.filter.Local_point_y[i]) - 4, 8,
                        8, blackPen1);
    }
  }
  {
    constexpr double ROBOT_DETECT_RANGE_PX = 200.0;
    constexpr double MAP_MATCH_THRESHOLD_PX = 80.0;
    QPen nearbyPen(Qt::cyan);
    nearbyPen.setWidth(3);
    QBrush nearbyBrush(Qt::cyan);

    for (int i = 0; i < (int)localization_snapshot_.particle_filter.vision_points.size(); i++)
    {
      auto &vp = localization_snapshot_.particle_filter.vision_points[i];

      double dist_from_robot = sqrt(pow(vp.POINT_VEC_X, 2) + pow(vp.POINT_VEC_Y, 2));
      if (dist_from_robot > ROBOT_DETECT_RANGE_PX)
        continue;

      int obs_px = VisionPointUtils::worldX(vp);
      int obs_py = VisionPointUtils::worldY(vp);

      double min_dis = MAP_MATCH_THRESHOLD_PX;
      int nearest_j = -1;
      for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
      {
        if ((int)FieldCoord::ALL_POINTS[j].type != vp.FEATURE_TYPE)
          continue;
        double dis = sqrt(pow(localization_snapshot_.particle_filter.filter.Local_point_x[j] - obs_px, 2) +
                          pow(localization_snapshot_.particle_filter.filter.Local_point_y[j] - obs_py, 2));
        if (dis < min_dis)
        {
          min_dis = dis;
          nearest_j = j;
        }
      }
      if (nearest_j >= 0)
      {
        scene->addEllipse(cvt_Print_xy(localization_snapshot_.particle_filter.filter.Local_point_x[nearest_j]) - 6,
                          cvt_Print_xy(localization_snapshot_.particle_filter.filter.Local_point_y[nearest_j]) - 6,
                          12, 12, nearbyPen, nearbyBrush);
      }
    }
  }

  for (int i = 0; i < localization_snapshot_.particle_filter.vision_points.size(); i++)
  {
    auto px = cvt_Print_xy(localization_snapshot_.particle_filter.vision_points[i].STD_X +
                           localization_snapshot_.particle_filter.vision_points[i].POINT_VEC_X);
    auto py = cvt_Print_xy(localization_snapshot_.particle_filter.vision_points[i].STD_Y +
                           localization_snapshot_.particle_filter.vision_points[i].POINT_VEC_Y);
    if (localization_snapshot_.particle_filter.vision_points[i].CONFIDENCE > 0.9)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen1);
    }
    else if (localization_snapshot_.particle_filter.vision_points[i].CONFIDENCE > 0.7)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen2);
    }
    else if (localization_snapshot_.particle_filter.vision_points[i].CONFIDENCE > 0.5)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen3);
    }
  }
  if (localization_snapshot_.particle_filter.vision_points.size() != 0)
  {

    scene->addEllipse(cvt_Print_xy(localization_snapshot_.particle_filter.filter.CIRCLE_CENTER.x -
                                   localization_snapshot_.particle_filter.filter.CIRCLE_R),
                      cvt_Print_xy(localization_snapshot_.particle_filter.filter.CIRCLE_CENTER.y -
                                   localization_snapshot_.particle_filter.filter.CIRCLE_R),
                      cvt_Print_xy(localization_snapshot_.particle_filter.filter.CIRCLE_R * 2),
                      cvt_Print_xy(localization_snapshot_.particle_filter.filter.CIRCLE_R * 2), blackPen);
  }

  if (localization_snapshot_.master.master_best_position_attack_x != 0.0f || localization_snapshot_.master.master_best_position_attack_y != 0.0f)
  {
    int bpx = FieldCoord::toPixelX(localization_snapshot_.master.master_best_position_attack_x);
    int bpy = FieldCoord::toPixelY(localization_snapshot_.master.master_best_position_attack_y);
    QPen bluePen(Qt::blue);
    bluePen.setWidth(2);
    QBrush blueFill(Qt::blue);
    scene->addEllipse(bpx - 8, bpy - 8, 16, 16, bluePen, blueFill);
  }

  if (localization_snapshot_.master.master_best_position_defend_x != 0.0f ||
      localization_snapshot_.master.master_best_position_defend_y != 0.0f)
  {
    int bpx = FieldCoord::toPixelX(localization_snapshot_.master.master_best_position_defend_x);
    int bpy = FieldCoord::toPixelY(localization_snapshot_.master.master_best_position_defend_y);
    QPen purplePen(Qt::magenta);
    purplePen.setWidth(2);

    QBrush purpleFill(Qt::magenta);

    scene->addEllipse(bpx - 8, bpy - 8, 16, 16, purplePen, purpleFill);
  }

  if (localization_snapshot_.master.master_target_x != 0 || localization_snapshot_.master.master_target_y != 0)
  {
    scene->addLine(cvt_Print_xy(localization_snapshot_.master.master_target_x) + 4,
                   cvt_Print_xy(localization_snapshot_.master.master_target_y),
                   cvt_Print_xy(localization_snapshot_.master.master_target_x) - 4,
                   cvt_Print_xy(localization_snapshot_.master.master_target_y), targetPen);
    scene->addLine(cvt_Print_xy(localization_snapshot_.master.master_target_x),
                   cvt_Print_xy(localization_snapshot_.master.master_target_y) + 4,
                   cvt_Print_xy(localization_snapshot_.master.master_target_x),
                   cvt_Print_xy(localization_snapshot_.master.master_target_y) - 4, targetPen);
    scene->addLine(cvt_Print_xy(localization_snapshot_.master.master_target_x) + 2,
                   cvt_Print_xy(localization_snapshot_.master.master_target_y) + 2,
                   cvt_Print_xy(localization_snapshot_.robot.pose.x), cvt_Print_xy(localization_snapshot_.robot.pose.y),
                   targetlinePen);

    if (!localization_snapshot_.master.master_move2ball_phase.empty())
    {
      float att = localization_snapshot_.master.master_angle_to_target;
      double theta = (270.0 - att) * DEG2RAD;
      double dir_x = std::cos(theta);
      double dir_y = std::sin(theta);

      if (std::abs(dir_x) > 1e-6)
      {
        using namespace FieldCoord;
        int goal_line_x = (dir_x > 0.0) ? toPixelX(A / 2) : toPixelX(-A / 2);
        double t = (goal_line_x - localization_snapshot_.master.master_target_x) / dir_x;
        if (t > 0.0)
        {
          double aim_y_raw = localization_snapshot_.master.master_target_y + t * dir_y;
          int goal_y_min = toPixelY(D / 2);
          int goal_y_max = toPixelY(-D / 2);
          int aim_y = static_cast<int>(std::max((double)goal_y_min, std::min((double)goal_y_max, aim_y_raw)));

          QPen aimDotPen(QColor(255, 165, 0));
          QBrush aimDotBrush(QColor(255, 165, 0));
          scene->addEllipse(goal_line_x - 5, aim_y - 5, 10, 10, aimDotPen, aimDotBrush);
        }
      }
    }
  }

  bool teammate_in_state_three = false;
  for (const auto& teammate : localization_snapshot_.world.teammates)
  {
    teammate_in_state_three |= teammate.active && teammate.state == 3;
  }

  QPolygonF robot0_poly = create_Print_robot(
      localization_snapshot_.robot.pose.x, localization_snapshot_.robot.pose.y,
      localization_snapshot_.robot.pose.yaw_deg);
  const BALL& ball = localization_snapshot_.world.ball;
  if (!teammate_in_state_three && (ball.x != 0 || ball.y != 0))
  {
    scene->addPolygon(robot0_poly, redPen3, blueBrush);
  }
  else
  {
    scene->addPolygon(robot0_poly, blackPen, blueBrush);
  }

  scene->addLine(
      cvt_Print_xy(localization_snapshot_.robot.pose.x), cvt_Print_xy(localization_snapshot_.robot.pose.y),
      cvt_Print_xy(localization_snapshot_.robot.pose.x) +
          sin((-1) * localization_snapshot_.robot.pose.yaw_deg * DEG2RAD) * 10,
      cvt_Print_xy(localization_snapshot_.robot.pose.y) -
          cos(localization_snapshot_.robot.pose.yaw_deg * DEG2RAD) * 10,
      blackPen3);

  const std::array<QBrush, 4> teammate_brushes = {
      redBrush, greenBrush, yellowBrush, blackBrush};

  for (std::size_t i = 0;
       i < localization_snapshot_.world.teammates.size(); ++i)
  {
    const auto& teammate = localization_snapshot_.world.teammates[i];
    if (!teammate.active)
    {
      continue;
    }

    const QPolygonF teammate_polygon = create_Print_robot(
        teammate.pose.x, teammate.pose.y, teammate.pose.yaw_deg);
    const QPen& teammate_pen = teammate.state == 3 ? redPen3 : blackPen;
    scene->addPolygon(
        teammate_polygon, teammate_pen, teammate_brushes[i]);
    scene->addLine(
        cvt_Print_xy(teammate.pose.x), cvt_Print_xy(teammate.pose.y),
        cvt_Print_xy(teammate.pose.x) +
            sin((-1) * teammate.pose.yaw_deg * DEG2RAD) * 10,
        cvt_Print_xy(teammate.pose.y) -
            cos(teammate.pose.yaw_deg * DEG2RAD) * 10,
        blackPen3);
  }

  Print_Screen_Ball();
}

void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
  QPointF scenePos;
  if (!mapToFieldScene(*ui->screen, *this, e->pos(), scenePos))
  {
    return;
  }
  if (set_robot_flag)
  {
    commands_->placeRobot(scenePos.x(), scenePos.y(), particle_range);
    set_robot_flag = 0;
  }

  if (!obstacle_mode && manual_ball_editing_)
  {
    commands_->setManualBall(scenePos.x(), scenePos.y());
    manual_ball_placed_ = true;
  }

  if (set_obstacle_flag_)
  {
    commands_->addManualObstacle(scenePos.x(), scenePos.y());
    set_obstacle_flag_ = false;
  }
}

void MainWindow::mouseMoveEvent(QMouseEvent *e)
{
  QPointF scenePos;
  if (!mapToFieldScene(*ui->screen, *this, e->pos(), scenePos))
  {
    return;
  }
  if (set_robot_flag)
  {
    commands_->previewRobotPosition(scenePos.x(), scenePos.y());
  }
  if (!obstacle_mode && manual_ball_editing_)
  {
    commands_->setManualBall(scenePos.x(), scenePos.y());
    manual_ball_placed_ = true;
  }
}

QPolygonF MainWindow::create_Print_robot(
    float x, float y, float yaw_deg)
{
  float Robot_model_x = x;
  float Robot_model_y = y;
  float Robot_model_z = yaw_deg + 90;

  float robot_shape_1_x = -12, robot_shape_1_y = 9,
        robot_shape_2_x = 12, robot_shape_2_y = 9,
        robot_shape_3_x = -12, robot_shape_3_y = -9,
        robot_shape_4_x = 12, robot_shape_4_y = -9;

  QPolygonF Robot_model_poly;
  Robot_model_poly
      << QPointF((robot_shape_1_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_1_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_1_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_1_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_2_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_2_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_2_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_2_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_4_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_4_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_4_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_4_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y)
      << QPointF((robot_shape_3_x)*sin(Robot_model_z * M_PI / 180) -
                     (robot_shape_3_y)*cos(Robot_model_z * M_PI / 180) +
                     Robot_model_x,
                 (robot_shape_3_x)*cos(Robot_model_z * M_PI / 180) +
                     (robot_shape_3_y)*sin(Robot_model_z * M_PI / 180) +
                     Robot_model_y);

  return Robot_model_poly;
}

int MainWindow::cvt_Print_xy(float target)
{
  return (int)target;
}

void MainWindow::on_btn_free_set_clicked()
{
  set_robot_flag = 1;
}
void MainWindow::on_btn_objects_save_clicked()
{
  // Write to the source tree: install/ is recopied from source on every build.
  const std::string rel = robocup_localization::app::odomParamRelativePath();
  const std::string param =
      rel.empty() ? std::string()
                  : robocup_localization::app::resourcePath(rel);

  ofstream Last_Index_Num_OUT(param);
  if (Last_Index_Num_OUT.is_open())
  {
    Last_Index_Num_OUT << particle_range << endl;
    Last_Index_Num_OUT << ui->odom_fx_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_bx_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_ly_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_ry_value->value() << endl;
    printf("[LocalNode] Odom Param saved: %s\n", param.c_str());
  }
  else
  {
    printf("[LocalNode] Odom Param save FAILED: %s\n",
           param.empty() ? "로봇 번호 미정의(robot_config.h)" : param.c_str());
  }
  Last_Index_Num_OUT.close();
}

static void setRobotPos(
    robocup_localization::app::LocalizationCommand* commands,
    float x, float y, float particle_range)
{
  commands->placeRobot(x, y, particle_range);
}

void MainWindow::on_btn_set_1_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(0.8), toPixelY(B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-3.5), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_2_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(2.5), toPixelY(B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-2.5), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_3_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(3.5), toPixelY(B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-0.8), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_4_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(0.8), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-3.5), toPixelY(-B / 2), particle_range);
}
void MainWindow::on_btn_set_5_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(2.5), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-2.5), toPixelY(-B / 2), particle_range);
}
void MainWindow::on_btn_set_6_clicked()
{
  using namespace FieldCoord;
  if (localization_snapshot_.game.mySide)
    setRobotPos(commands_, toPixelX(3.5), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(commands_, toPixelX(-0.8), toPixelY(-B / 2), particle_range);
}

void MainWindow::on_btn_set_auto_clicked()
{
  using namespace FieldCoord;

  const auto pose = localization_config_.initial_pose.forSide(
      localization_snapshot_.game.robotNum,
      localization_snapshot_.game.mySide);

  setRobotPos(
      commands_, toPixelX(pose.x), toPixelY(pose.y), particle_range);
}
void MainWindow::on_btn_ball_set_clicked()
{
  if (obstacle_mode)
  {
    set_obstacle_flag_ = true;
  }
  else
  {
    manual_ball_editing_ = true;
    manual_ball_placed_ = false;
  }
}
void MainWindow::on_btn_ball_del_clicked()
{
  if (obstacle_mode)
  {
    commands_->removeLastManualObstacle();
  }
  else
  {
    manual_ball_editing_ = false;
    manual_ball_placed_ = false;
    commands_->clearManualBall();
  }
}
void MainWindow::on_obstacle_trigger_clicked()
{
  obstacle_mode = !obstacle_mode;
  if (obstacle_mode)
    ui->subtitle0_2->setText("Set Obstacle");
  else
    ui->subtitle0_2->setText(" SET BALL");
}
void MainWindow::on_btn_test_clicked()
{
  commands_->clearTeammates();
  commands_->clearParticleFilterVision();
}

void MainWindow::on_btn_info_clicked()
{
  Popup dialog(this);
  dialog.exec();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow() { delete ui; }
