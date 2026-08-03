/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "robocup_localization/ui/main_window.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <QMessageBox>
#include <algorithm>
#include <array>
#include <cmath>
#include <QtGui>
#include <fstream>
#include <iostream>
#include <mutex>
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

}  // namespace

MainWindow::MainWindow(
    robocup_localization::app::LocalizationSnapshotSource& snapshots,
    robocup_localization::app::LocalizationCommand& commands,
    QWidget* parent)
    : QMainWindow(parent),
      snapshots_(&snapshots),
      commands_(&commands),
      localization_snapshot_(snapshots.localizationSnapshot()),
      ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  scaleMainWindowUi(*this, *ui->centralwidget);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  const auto config = snapshots_->localizationConfig();
  const auto interval_ms = [](double hz)
  {
    const double safe_hz = std::max(0.1, hz);
    return std::max(1, static_cast<int>(std::lround(1000.0 / safe_hz)));
  };

  // 데이터 업데이트 타이머
  m_DataTimer = new QTimer(this);
  connect(m_DataTimer, SIGNAL(timeout()), this, SLOT(main()));
  m_DataTimer->start(interval_ms(config.ui_data_hz));

  // UI 업데이트 타이머
  m_screenTimer = new QTimer(this);
  connect(m_screenTimer, SIGNAL(timeout()), this, SLOT(Print_Screen()));
  m_screenTimer->start(interval_ms(config.ui_render_hz));

  // 맵 QPixmap 동적 생성 (FieldCoord 기반)
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

    // 외곽 경계선
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::B / 2),
               (int)(FieldCoord::A * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::B * FieldCoord::PX_PER_M_Y));

    // 하프라인
    p.drawLine(FieldCoord::toPixelX(0), FieldCoord::toPixelY(FieldCoord::B / 2),
               FieldCoord::toPixelX(0), FieldCoord::toPixelY(-FieldCoord::B / 2));

    // 센터 서클 (J = 지름)
    int circle_r = (int)(FieldCoord::J / 2.0 * FieldCoord::PX_PER_M_X);
    p.drawEllipse(FieldCoord::toPixelX(0) - circle_r,
                  FieldCoord::toPixelY(0) - circle_r,
                  circle_r * 2, circle_r * 2);

    // 골대 (필드 경계 바깥, C 깊이 × D 너비)
    int goal_w = (int)(FieldCoord::C * FieldCoord::PX_PER_M_X);
    int goal_h = (int)(FieldCoord::D * FieldCoord::PX_PER_M_Y);
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2) - goal_w,
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);

    // 좌측 골 에어리어
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M_Y));

    // 우측 골 에어리어
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::E),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M_Y));

    // 좌측 페널티 에어리어
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M_Y));

    // 우측 페널티 에어리어
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::G),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M_X),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M_Y));

    // 마크 크기 (픽셀)
    int mark_sz = (int)(FieldCoord::MARK_SZ / 2.0 * FieldCoord::PX_PER_M_X);
    if (mark_sz < 2)
      mark_sz = 2;

    p.setBrush(Qt::white);

    // 센터 마크 (점)
    p.drawEllipse(FieldCoord::toPixelX(0) - mark_sz, FieldCoord::toPixelY(0) - mark_sz,
                  mark_sz * 2, mark_sz * 2);

    // 좌측 페널티 마크 (점)
    int pm0_x = FieldCoord::toPixelX(-FieldCoord::A / 2 + FieldCoord::I_DIST);
    int pm0_y = FieldCoord::toPixelY(0);
    p.drawEllipse(pm0_x - mark_sz, pm0_y - mark_sz, mark_sz * 2, mark_sz * 2);

    // 우측 페널티 마크 (점)
    int pm1_x = FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::I_DIST);
    int pm1_y = FieldCoord::toPixelY(0);
    p.drawEllipse(pm1_x - mark_sz, pm1_y - mark_sz, mark_sz * 2, mark_sz * 2);

    p.setBrush(Qt::NoBrush);

    // 코너 아크 (L_ARC > 0인 경우만)
    if (FieldCoord::L_ARC > 0.0)
    {
      int r = (int)(FieldCoord::L_ARC * FieldCoord::PX_PER_M_X);
      int x_l = FieldCoord::toPixelX(-FieldCoord::A / 2);
      int x_r = FieldCoord::toPixelX(FieldCoord::A / 2);
      int y_t = FieldCoord::toPixelY(FieldCoord::B / 2);
      int y_b = FieldCoord::toPixelY(-FieldCoord::B / 2);
      p.drawArc(x_l - r, y_t - r, 2 * r, 2 * r, 270 * 16, 90 * 16); // 좌상단
      p.drawArc(x_r - r, y_t - r, 2 * r, 2 * r, 180 * 16, 90 * 16); // 우상단
      p.drawArc(x_l - r, y_b - r, 2 * r, 2 * r, 0 * 16, 90 * 16);   // 좌하단
      p.drawArc(x_r - r, y_b - r, 2 * r, 2 * r, 90 * 16, 90 * 16);  // 우하단
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

    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("robocup_localization");
    String param = "";
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
    param = share_dir + "/resources/param/PARAM1.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
    param = share_dir + "/resources/param/PARAM2.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
    param = share_dir + "/resources/param/PARAM3.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
    param = share_dir + "/resources/param/PARAM4.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
    param = share_dir + "/resources/param/PARAM5.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
    param = share_dir + "/resources/param/PARAM6.txt";
#endif

    ifstream input(param);
    if (input.is_open())
    {
      input >> particle_range;
      input >> odom_forward;
      input >> odom_backward;
      input >> odom_left;
      input >> odom_right;
      printf("[LocalNode] Odom Load\n");
    }
    else
    {
      printf("[LocalNode] No Odom Data\n");
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

void MainWindow::Print_Screen() // ui 출력, 200ms마다 실행, 공 출력은 50ms로 분리
{
  localization_snapshot_ = snapshots_->localizationSnapshot();
  if (manual_ball_editing_ && manual_ball_placed_ &&
      !localization_snapshot_.world.manual_ball_active)
  {
    manual_ball_editing_ = false;
    manual_ball_placed_ = false;
  }
  // PRE CONDITION : robotX.n, obstacleX.n, qnode->ball.n, ParticleFilter
  // POST CONDITION : UI
  // purpose : UI출력

  // INIT
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

  // print obstacle
  for (const auto &obs : localization_snapshot_.world.obstacles)
  {
    if (obs.x != 0 && obs.y != 0)
    {
      scene->addRect(cvt_Print_xy(obs.x) - 5,
                     cvt_Print_xy(obs.y) - 5, 10, 10, blackPen,
                     blackBrush);
    }
  }

  // print debug obstacles
  for (const auto &obs : localization_snapshot_.world.manual_obstacles)
  {
    scene->addRect(cvt_Print_xy(obs.x) - 6,
                   cvt_Print_xy(obs.y) - 6, 12, 12, redPen,
                   redBrush);
  }

  // PRINT PARTICLE
  for (const auto& particle : localization_snapshot_.particle_filter.particles)
  {
    scene->addLine(
        cvt_Print_xy(particle.x), cvt_Print_xy(particle.y),
        cvt_Print_xy(particle.x), cvt_Print_xy(particle.y), grayPen);
  }

  // PRINT TIMESTAMP
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

  // PRINT SIGHT

  // 비전에서 측정된 특징점 표시
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
  // 로봇 기준 일정 거리 내 관측 특징점과 가장 가까운 로컬 맵 특징점 좌표 하이라이트
  {
    constexpr double ROBOT_DETECT_RANGE_PX = 200.0; // 로봇~특징점 최대 거리 (픽셀, 조정 가능)
    constexpr double MAP_MATCH_THRESHOLD_PX = 80.0; // 관측점~맵점 매칭 허용 오차 (픽셀)
    QPen nearbyPen(Qt::cyan);
    nearbyPen.setWidth(3);
    QBrush nearbyBrush(Qt::cyan);

    for (int i = 0; i < (int)localization_snapshot_.particle_filter.vision_points.size(); i++)
    {
      auto &vp = localization_snapshot_.particle_filter.vision_points[i];

      // 로봇에서 특징점까지의 거리 (픽셀)
      double dist_from_robot = sqrt(pow(vp.POINT_VEC_X, 2) + pow(vp.POINT_VEC_Y, 2));
      if (dist_from_robot > ROBOT_DETECT_RANGE_PX)
        continue;

      // 관측 특징점의 절대 픽셀 좌표
      int obs_px = VisionPointUtils::worldX(vp);
      int obs_py = VisionPointUtils::worldY(vp);

      // 같은 타입의 로컬 맵 특징점 중 가장 가까운 것 찾기
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

  // PRINT BEST ATTACK POSITION
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

  // PRINT MASTER TARGET
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

    // angle_to_target 방향과 골대 라인 교차점 표시 (GO_KICK 중에만)
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

  // PRINT ROBOT
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
  // PRE CONDITION : mouseEvent, set_robot_flag, manual_ball_editing_
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 클릭 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성

  // MainWindow 기준 마우스 위치 → ui->screen(QGraphicsView) 기준 → 씬 좌표
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
  // PRE CONDITION : mouseEvent, set_robot_flag, manual_ball_editing_
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 이동 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성
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
  // PRE CONDITION : robot.n, zoom(0.75)
  // POST CONDITION : Robot_model_poly
  // purpose : 로봇의 x, y, z좌표에 zoom값을 곱한 후 해당 값에 의한 폴리곤 생성

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
  // 좌표계가 픽셀 단위로 변경됨 - 변환 없이 그대로 반환
  return (int)target;
}

void MainWindow::on_btn_free_set_clicked()
{
  // PRE CONDITION : UI
  // POST CONDITION : set_robot_flag
  // purpose : set_robot_flag를 1로 설정

  set_robot_flag = 1;
}
void MainWindow::on_btn_objects_save_clicked()
{
  // PRE CONDITION : particle_range, robot0.odom_Nn
  // POST CONDITION : param file
  // purpose : 현재 적용되어있는 오도메트리 값을 입출력스트림을 통해 저장

  std::string share_dir = ament_index_cpp::get_package_share_directory("robocup_localization");
  String param = "";
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
  param = share_dir + "/resources/param/PARAM1.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
  param = share_dir + "/resources/param/PARAM2.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
  param = share_dir + "/resources/param/PARAM3.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
  param = share_dir + "/resources/param/PARAM4.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
  param = share_dir + "/resources/param/PARAM5.txt";
#endif
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
  param = share_dir + "/resources/param/PARAM6.txt";
#endif
  ofstream Last_Index_Num_OUT(param);
  if (Last_Index_Num_OUT.is_open())
  {
    Last_Index_Num_OUT << particle_range << endl;
    Last_Index_Num_OUT << ui->odom_fx_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_bx_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_ly_value->value() << endl;
    Last_Index_Num_OUT << ui->odom_ry_value->value() << endl;
    printf("[LocalNode] Odom Param set\n");
  }
  else
  {
    printf("[LocalNode] No Odom Parameter\n");
  }
  Last_Index_Num_OUT.close();
}

// 로봇 위치 설정 헬퍼 (좌표 설정 + 파티클 재생성 + 타임스탬프 초기화)
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
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 자동 매크로
  using namespace FieldCoord;

  float x = 0, y = 0;
  if (localization_snapshot_.game.mySide)
  {
    if (localization_snapshot_.game.robotNum == 1)
    {

      x = toPixelX(3.5);
      y = toPixelY(-B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 2)
    {

      x = toPixelX(0.8);
      y = toPixelY(-B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 3)
    {
      x = toPixelX(2.5);
      y = toPixelY(B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 4)
    {
      x = toPixelX(0.8);
      y = toPixelY(B / 2);
    }
  }
  else
  {
    if (localization_snapshot_.game.robotNum == 1)
    {

      x = toPixelX(-3.5);
      y = toPixelY(B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 2)
    {
      x = toPixelX(-0.8);
      y = toPixelY(B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 3)
    {

      x = toPixelX(-2.5);
      y = toPixelY(-B / 2);
    }
    else if (localization_snapshot_.game.robotNum == 4)
    {
      x = toPixelX(-0.8);
      y = toPixelY(-B / 2);
    }
  }
  setRobotPos(commands_, x, y, particle_range);
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
