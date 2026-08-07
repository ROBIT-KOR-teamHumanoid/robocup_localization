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

#include "gui/main_window.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <QMessageBox>
#include <QtGui>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

#include "robot_config.h"
#include "field_profile/field_coordinate.hpp"
#include "field_profile/start_positions.hpp"
#include "utils/team_udp_receiver.hpp"
#include "gui/odom_param_store.hpp"
#include "pre_processing/vision_processing.hpp"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  // 데이터 업데이트 타이머
  m_DataTimer = new QTimer(this);
  connect(m_DataTimer, SIGNAL(timeout()), this, SLOT(main()));
  m_DataTimer->start(50);

  // UI 업데이트 타이머
  m_screenTimer = new QTimer(this);
  connect(m_screenTimer, SIGNAL(timeout()), this, SLOT(Print_Screen()));
  m_screenTimer->start(200);

  // 맵 QPixmap 동적 생성 (FieldCoord 기반)
  buf = QPixmap(FieldCoord::MAP_W, FieldCoord::MAP_H);
  buf.fill(QColor(34, 139, 34));

  {
    QPainter p(&buf);
    p.setRenderHint(QPainter::Antialiasing);

    int line_w = (int)(FieldCoord::LINE_W * FieldCoord::PX_PER_M);
    if (line_w < 1)
      line_w = 1;

    p.setPen(QPen(Qt::white, line_w));
    p.setBrush(Qt::NoBrush);

    // 외곽 경계선
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::B / 2),
               (int)(FieldCoord::A * FieldCoord::PX_PER_M),
               (int)(FieldCoord::B * FieldCoord::PX_PER_M));

    // 하프라인
    p.drawLine(FieldCoord::toPixelX(0), FieldCoord::toPixelY(FieldCoord::B / 2),
               FieldCoord::toPixelX(0), FieldCoord::toPixelY(-FieldCoord::B / 2));

    // 센터 서클 (J = 지름)
    int circle_r = (int)(FieldCoord::J / 2.0 * FieldCoord::PX_PER_M);
    p.drawEllipse(FieldCoord::toPixelX(0) - circle_r,
                  FieldCoord::toPixelY(0) - circle_r,
                  circle_r * 2, circle_r * 2);

    // 골대 (필드 경계 바깥, C 깊이 × D 너비)
    int goal_w = (int)(FieldCoord::C * FieldCoord::PX_PER_M);
    int goal_h = (int)(FieldCoord::D * FieldCoord::PX_PER_M);
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2) - goal_w,
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::D / 2),
               goal_w, goal_h);

    // 좌측 골 에어리어
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M));

    // 우측 골 에어리어
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::E),
               FieldCoord::toPixelY(FieldCoord::F / 2),
               (int)(FieldCoord::E * FieldCoord::PX_PER_M),
               (int)(FieldCoord::F * FieldCoord::PX_PER_M));

    // 좌측 페널티 에어리어
    p.drawRect(FieldCoord::toPixelX(-FieldCoord::A / 2),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M));

    // 우측 페널티 에어리어
    p.drawRect(FieldCoord::toPixelX(FieldCoord::A / 2 - FieldCoord::G),
               FieldCoord::toPixelY(FieldCoord::H / 2),
               (int)(FieldCoord::G * FieldCoord::PX_PER_M),
               (int)(FieldCoord::H * FieldCoord::PX_PER_M));

    // 마크 크기 (픽셀)
    int mark_sz = (int)(FieldCoord::MARK_SZ / 2.0 * FieldCoord::PX_PER_M);
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
      int r = (int)(FieldCoord::L_ARC * FieldCoord::PX_PER_M);
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
  // 로봇 시작 포인트, 필드 중앙
  qnode->mcl_.moveRobot(FieldCoord::toPixelX(0), FieldCoord::toPixelY(0));

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::main()
{
  // 추적 갱신과 발행은 QNode 의 ROS 타이머가 한다 (GUI 가 멈춰도 계속되도록).
  // 여기서는 슬라이더 값을 반영하는 일만 남는다.
  setting();
}

void MainWindow::Print_Screen_Ball()
{
  if (!scene)
    return;
  QPen blackPen(Qt::black);
  QBrush redBrush(Qt::red);
  QBrush yellowBrush(Qt::yellow);
  QBrush blueBrush(Qt::blue);
  QBrush pinkBrush(QColor(255, 105, 180));

  // 그리기 전 한 번 복사 — ROS 타이머가 갱신 중일 수 있다.
  const BallTracker::Snapshot ts = qnode->tracker_.snapshot();

  bool ball_in_range = (ts.ball.x >= FieldCoord::BALL_TRUST_X_MIN &&
                        ts.ball.x <= FieldCoord::BALL_TRUST_X_MAX &&
                        ts.ball.y >= FieldCoord::BALL_TRUST_Y_MIN &&
                        ts.ball.y <= FieldCoord::BALL_TRUST_Y_MAX);
  if (ball_in_range && (qnode->visionMSG.Ball_2d_X != 0 || qnode->visionMSG.Ball_2d_Y != 0)
      && ts.ball.noballcnt == 0)
  {
    scene->addEllipse(cvt_Print_xy(ts.ball.x) - 5,
                      cvt_Print_xy(ts.ball.y) - 5, 11, 11, blackPen, redBrush);
    scene->addLine(cvt_Print_xy(ts.ball.x), cvt_Print_xy(ts.ball.y),
                   cvt_Print_xy(ts.ball.x + ts.ball.speed_x * qnode->visionMSG.Ball_speed_level),
                   cvt_Print_xy(ts.ball.y + ts.ball.speed_y * qnode->visionMSG.Ball_speed_level),
                   blackPen);
  }
  else if (ball_in_range && (ts.ball.x != 0 || ts.ball.y != 0))
  {
    if (ts.ball.from_udp)
      scene->addEllipse(cvt_Print_xy(ts.ball.x) - 5,
                        cvt_Print_xy(ts.ball.y) - 5, 11, 11, blackPen, pinkBrush);
    else
      scene->addEllipse(cvt_Print_xy(ts.ball.x) - 5,
                        cvt_Print_xy(ts.ball.y) - 5, 11, 11, blackPen, yellowBrush);
  }
  if (qnode->set_ball_flag)
  {
    scene->addEllipse(cvt_Print_xy(ts.ball.x) - 5,
                      cvt_Print_xy(ts.ball.y) - 5, 11, 11, blackPen, blueBrush);
  }
}


void MainWindow::setting()
{
  // PRE CONDITION : qnode->vision_callback_timer, qnode->set_ball_flag,
  // ball_set_N, setting_flag, 파라미터 저장파일 POST CONDITION : ball_N,
  // qnode->robot0.odom_Nn purpose : 로봇의 초기 세팅 및 입출력스트림을 통해
  // 저장되어있던 오도메트리 값 설정, 볼의 위치를 재설정 하는 경우 볼 위치 변경,
  // ui슬라이더를 통해 오도메트리 값 적용

  if (setting_flag == 0) // 최초 1회: 저장된 캘리브레이션을 불러와 슬라이더에 반영
  {
    param_store::OdomParams p;
    param_store::load(p);   // 파일이 없으면 기본값 유지

    particle_range = p.particle_range;
    qnode->mcl_.motion.config.odom_scale_forward = p.odom_scale_forward;
    qnode->mcl_.motion.config.odom_scale_backward = p.odom_scale_backward;
    qnode->mcl_.motion.config.odom_scale_left = p.odom_scale_left;
    qnode->mcl_.motion.config.odom_scale_right = p.odom_scale_right;

    ui->particle_range_value->setValue(particle_range);
    ui->odom_fx_value->setValue(p.odom_scale_forward);
    ui->odom_bx_value->setValue(p.odom_scale_backward);
    ui->odom_ly_value->setValue(p.odom_scale_left);
    ui->odom_ry_value->setValue(p.odom_scale_right);
    setting_flag = 1;
  }
  // ui의 슬라이더 값을 오도메트리 값에 적용
  particle_range = ui->particle_range_value->value();
  qnode->particle_range = particle_range;
  qnode->mcl_.motion.config.odom_scale_forward = ui->odom_fx_value->value();
  qnode->mcl_.motion.config.odom_scale_backward = ui->odom_bx_value->value();
  qnode->mcl_.motion.config.odom_scale_left = ui->odom_ly_value->value();
  qnode->mcl_.motion.config.odom_scale_right = ui->odom_ry_value->value();
}

void MainWindow::Print_Screen() // ui 출력, 200ms마다 실행, 공 출력은 50ms로 분리
{
  // 그리기 전 추정 상태를 한 번에 복사한다 (일관된 시점 + 락 보유 최소화).
  const MclNode::Snapshot snap = qnode->mcl_.snapshot();
  const BallTracker::Snapshot ts = qnode->tracker_.snapshot();
  const QNode::TeamSnapshot team = qnode->teammates();
  const MasterMsg master = qnode->masterState();

  // PRE CONDITION : robotX.n, obstacleX.n, ts.ball.n, qnode->Likelihood
  // POST CONDITION : UI
  // purpose : UI출력

  // INIT
  if (scene)
  {
    delete scene;
  }

  char TEXT[256];
  sprintf(TEXT, "BALL_X : %d", (int)ts.ball.x);
  ui->BALL_X_value->setText(TEXT);
  sprintf(TEXT, "BALL_Y : %d", (int)ts.ball.y);
  ui->BALL_Y_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_X : %d", (int)snap.robot.x);
  ui->ROBOT_X_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_Y : %d", (int)snap.robot.y);
  ui->ROBOT_Y_value->setText(TEXT);
  sprintf(TEXT, "IK_X : %d", (int)snap.robot.last_ik_cmd_x);
  ui->IK_X_value->setText(TEXT);
  sprintf(TEXT, "IK_Y : %d", (int)snap.robot.last_ik_cmd_y);
  ui->IK_Y_value->setText(TEXT);

  scene = new QGraphicsScene(this);
  ui->screen->setScene(scene);
  scene->addPixmap(buf);

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
  for (const auto &obs : ts.obstacles)
  {
    if (obs.x != 0 && obs.y != 0)
    {
      scene->addRect(cvt_Print_xy(obs.x) - 5,
                     cvt_Print_xy(obs.y) - 5, 10, 10, blackPen,
                     blackBrush);
    }
  }

  // print debug obstacles
  for (const auto &obs : ts.debug_obstacles)
  {
    scene->addRect(cvt_Print_xy(obs.x) - 6,
                   cvt_Print_xy(obs.y) - 6, 12, 12, redPen,
                   redBrush);
  }

  // PRINT PARTICLE
  if (true)
  {
    for (int i = 0; i < qnode->particle_num; i++)
    {
      scene->addLine(cvt_Print_xy(snap.particles[i].x), cvt_Print_xy(snap.particles[i].y),
                     cvt_Print_xy(snap.particles[i].x), cvt_Print_xy(snap.particles[i].y),
                     grayPen);
    }
  }

  // ===== 학습형(ONNX) 추론 결과 시각화 =====
  {
    learned_loc::MlpResult mlp;
    learned_loc::RelocResult reloc;
    {
      std::lock_guard<std::mutex> lock(qnode->learned_mutex_);
      mlp = qnode->learned_mlp_result_;
      reloc = qnode->learned_reloc_result_;
    }

    // clsreg 재위치추정 후보: 청록 점(확률 클수록 크게)
    if (qnode->learned_reloc_enable_ && reloc.valid)
    {
      QPen cyanPen(QColor(0, 200, 200));
      cyanPen.setWidth(1);
      QBrush cyanBrush(QColor(0, 200, 200, 140));
      for (const auto &c : reloc.candidates)
      {
        double px = cvt_Print_xy(FieldCoord::toPixelX(c.x_m));
        double py = cvt_Print_xy(FieldCoord::toPixelY(c.y_m));
        int r = 2 + (int)(c.prob * 10.0);
        if (r > 8) r = 8;
        scene->addEllipse(px - r, py - r, r * 2, r * 2, cyanPen, cyanBrush);
      }
    }

    // vision_imu 단일프레임 pose: 자홍 원 + 십자
    if (qnode->learned_mlp_enable_ && mlp.valid)
    {
      QColor magenta(255, 0, 255);
      QPen mgPen(magenta);
      mgPen.setWidth(3);
      QBrush mgBrush(QColor(255, 0, 255, 90));
      double px = cvt_Print_xy(FieldCoord::toPixelX(mlp.x_m));
      double py = cvt_Print_xy(FieldCoord::toPixelY(mlp.y_m));
      int r = 10;
      scene->addEllipse(px - r, py - r, r * 2, r * 2, mgPen, mgBrush);
      scene->addLine(px - r, py, px + r, py, mgPen);
      scene->addLine(px, py - r, px, py + r, mgPen);
    }
  }

  // PRINT TIMESTAMP
  if (snap.robot.trail.size() > 0)
  {
    scene->addLine(
        cvt_Print_xy(snap.robot.x), cvt_Print_xy(snap.robot.y),
        cvt_Print_xy(
            snap.robot.trail[snap.robot.trail.size() - 1].x),
        cvt_Print_xy(
            snap.robot.trail[snap.robot.trail.size() - 1].y),
        blackPen3);
    for (int i = 1; i < snap.robot.trail.size(); i++)
    {
      scene->addLine(cvt_Print_xy(snap.robot.trail[i - 1].x),
                     cvt_Print_xy(snap.robot.trail[i - 1].y),
                     cvt_Print_xy(snap.robot.trail[i].x),
                     cvt_Print_xy(snap.robot.trail[i].y), blackPen3);
    }
  }

  // PRINT SIGHT
  // if(qnode->visionMSG.Scan_mode != 3){qnode->robot_sight_flag = 0;
  // snap.observations.clear();}

  // 비전에서 측정된 특징점 표시
  for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
  {
    if (!FieldCoord::ALL_POINTS[i].active)
      continue;
    if (snap.landmark_on[i] == 1)
    {
      scene->addEllipse(cvt_Print_xy(snap.landmark_x[i]) - 4,
                        cvt_Print_xy(snap.landmark_y[i]) - 4, 8,
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

    for (int i = 0; i < (int)snap.observations.size(); i++)
    {
      auto &vp = snap.observations[i];

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
        double dis = sqrt(pow(snap.landmark_x[j] - obs_px, 2) +
                          pow(snap.landmark_y[j] - obs_py, 2));
        if (dis < min_dis)
        {
          min_dis = dis;
          nearest_j = j;
        }
      }
      if (nearest_j >= 0)
      {
        scene->addEllipse(cvt_Print_xy(snap.landmark_x[nearest_j]) - 6,
                          cvt_Print_xy(snap.landmark_y[nearest_j]) - 6,
                          12, 12, nearbyPen, nearbyBrush);
      }
    }
  }

  for (int i = 0; i < snap.observations.size(); i++)
  {
    auto px = cvt_Print_xy(snap.observations[i].STD_X +
                           snap.observations[i].POINT_VEC_X);
    auto py = cvt_Print_xy(snap.observations[i].STD_Y +
                           snap.observations[i].POINT_VEC_Y);
    if (snap.observations[i].CONFIDENCE > 0.9)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen1);
    }
    else if (snap.observations[i].CONFIDENCE > 0.7)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen2);
    }
    else if (snap.observations[i].CONFIDENCE > 0.5)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen3);
    }
  }
  if (snap.observations.size() != 0)
  {

    scene->addEllipse(cvt_Print_xy(snap.circle_center.x -
                                   snap.circle_r),
                      cvt_Print_xy(snap.circle_center.y -
                                   snap.circle_r),
                      cvt_Print_xy(snap.circle_r * 2),
                      cvt_Print_xy(snap.circle_r * 2), blackPen);
  }

  // PRINT BEST ATTACK POSITION
  if (master.master_best_position_attack_x != 0.0f || master.master_best_position_attack_y != 0.0f)
  {
    int bpx = FieldCoord::toPixelX(master.master_best_position_attack_x);
    int bpy = FieldCoord::toPixelY(master.master_best_position_attack_y);
    QPen bluePen(Qt::blue);
    bluePen.setWidth(2);
    QBrush blueFill(Qt::blue);
    scene->addEllipse(bpx - 8, bpy - 8, 16, 16, bluePen, blueFill);
  }

  if (master.master_best_position_defend_x != 0.0f ||
      master.master_best_position_defend_y != 0.0f)
  {
    int bpx = FieldCoord::toPixelX(master.master_best_position_defend_x);
    int bpy = FieldCoord::toPixelY(master.master_best_position_defend_y);
    QPen purplePen(Qt::magenta);
    purplePen.setWidth(2);

    QBrush purpleFill(Qt::magenta);

    scene->addEllipse(bpx - 8, bpy - 8, 16, 16, purplePen, purpleFill);
  }

  // PRINT MASTER TARGET
  if (master.master_target_x != 0 || master.master_target_y != 0)
  {
    scene->addLine(cvt_Print_xy(master.master_target_x) + 4,
                   cvt_Print_xy(master.master_target_y),
                   cvt_Print_xy(master.master_target_x) - 4,
                   cvt_Print_xy(master.master_target_y), targetPen);
    scene->addLine(cvt_Print_xy(master.master_target_x),
                   cvt_Print_xy(master.master_target_y) + 4,
                   cvt_Print_xy(master.master_target_x),
                   cvt_Print_xy(master.master_target_y) - 4, targetPen);
    scene->addLine(cvt_Print_xy(master.master_target_x) + 2,
                   cvt_Print_xy(master.master_target_y) + 2,
                   cvt_Print_xy(snap.robot.x), cvt_Print_xy(snap.robot.y),
                   targetlinePen);

    // angle_to_target 방향과 골대 라인 교차점 표시 (GO_KICK 중에만)
    if (!master.master_move2ball_phase.empty())
    {
      float att = master.master_angle_to_target;
      double theta = (270.0 - att) * DEG2RAD;
      double dir_x = std::cos(theta);
      double dir_y = std::sin(theta);

      if (std::abs(dir_x) > 1e-6)
      {
        using namespace FieldCoord;
        int goal_line_x = (dir_x > 0.0) ? toPixelX(A / 2) : toPixelX(-A / 2);
        double t = (goal_line_x - master.master_target_x) / dir_x;
        if (t > 0.0)
        {
          double aim_y_raw = master.master_target_y + t * dir_y;
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
  QPolygonF robot0_poly = create_Print_robot(snap.robot);
  if (team.r[0].state != 3 && team.r[1].state != 3 &&
      team.r[2].state != 3 && team.r[3].state != 3 &&
      (ts.ball.x != 0 || ts.ball.y != 0))
  {
    scene->addPolygon(robot0_poly, redPen3, blueBrush);
  }
  else
  {
    scene->addPolygon(robot0_poly, blackPen, blueBrush);
  }
  scene->addLine(
      cvt_Print_xy(snap.robot.x), cvt_Print_xy(snap.robot.y),
      cvt_Print_xy(snap.robot.x) +
          sin((-1) * snap.robot.yaw_deg * DEG2RAD) * 10,
      cvt_Print_xy(snap.robot.y) - cos(snap.robot.yaw_deg * DEG2RAD) * 10,
      blackPen3);

  if (team.r[0].x != 0 || team.r[0].y != 0)
  {
    QPolygonF robot1_poly = create_Print_robot(team.r[0]);

    if (team.r[0].state == 3)
    {
      scene->addPolygon(robot1_poly, redPen3, redBrush);
    }
    else
    {
      scene->addPolygon(robot1_poly, blackPen, redBrush);
    }
    scene->addLine(
        cvt_Print_xy(team.r[0].x), cvt_Print_xy(team.r[0].y),
        cvt_Print_xy(team.r[0].x) +
            sin((-1) * team.r[0].yaw_deg * DEG2RAD) * 10,
        cvt_Print_xy(team.r[0].y) - cos(team.r[0].yaw_deg * DEG2RAD) * 10,
        blackPen3);
  }
  if (team.r[1].x != 0 || team.r[1].y != 0)
  {
    QPolygonF robot2_poly = create_Print_robot(team.r[1]);
    if (team.r[1].state == 3)
    {
      scene->addPolygon(robot2_poly, redPen3, greenBrush);
    }
    else
    {
      scene->addPolygon(robot2_poly, blackPen, greenBrush);
    }
    scene->addLine(
        cvt_Print_xy(team.r[1].x), cvt_Print_xy(team.r[1].y),
        cvt_Print_xy(team.r[1].x) +
            sin((-1) * team.r[1].yaw_deg * DEG2RAD) * 10,
        cvt_Print_xy(team.r[1].y) - cos(team.r[1].yaw_deg * DEG2RAD) * 10,
        blackPen3);
  }
  if (team.r[2].x != 0 || team.r[2].y != 0)
  {
    QPolygonF robot3_poly = create_Print_robot(team.r[2]);
    if (team.r[2].state == 3)
    {
      scene->addPolygon(robot3_poly, redPen3, yellowBrush);
    }
    else
    {
      scene->addPolygon(robot3_poly, blackPen, yellowBrush);
    }
    scene->addLine(
        cvt_Print_xy(team.r[2].x), cvt_Print_xy(team.r[2].y),
        cvt_Print_xy(team.r[2].x) +
            sin((-1) * team.r[2].yaw_deg * DEG2RAD) * 10,
        cvt_Print_xy(team.r[2].y) - cos(team.r[2].yaw_deg * DEG2RAD) * 10,
        blackPen3);
  }
  if (team.r[3].x != 0 || team.r[3].y != 0)
  {
    QPolygonF robot4_poly = create_Print_robot(team.r[3]);
    if (team.r[3].state == 3)
    {
      scene->addPolygon(robot4_poly, redPen3, blackBrush);
    }
    else
    {
      scene->addPolygon(robot4_poly, blackPen, blackBrush);
    }
    scene->addLine(
        cvt_Print_xy(team.r[3].x), cvt_Print_xy(team.r[3].y),
        cvt_Print_xy(team.r[3].x) +
            sin((-1) * team.r[3].yaw_deg * DEG2RAD) * 10,
        cvt_Print_xy(team.r[3].y) - cos(team.r[3].yaw_deg * DEG2RAD) * 10,
        blackPen3);
  }

  Print_Screen_Ball();
}

void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
  // PRE CONDITION : mouseEvent, set_robot_flag, qnode->set_ball_flag
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 클릭 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성

  // MainWindow 기준 마우스 위치 → ui->screen(QGraphicsView) 기준 → 씬 좌표
  QPointF scenePos = ui->screen->mapToScene(ui->screen->mapFrom(this, e->pos()));
  if (set_robot_flag)
  {
    qnode->mcl_.placeRobot(scenePos.x(), scenePos.y(), particle_range);
    set_robot_flag = 0;
    qnode->clearMasterTarget();
  }

  if (!obstacle_mode && qnode->set_ball_flag)
  {
    qnode->tracker_.setBall(scenePos.x(), scenePos.y());
  }

  if (qnode->set_obstacle_flag)
  {
    qnode->tracker_.addDebugObstacle(scenePos.x(), scenePos.y());
    qnode->set_obstacle_flag = 0;
  }
}

void MainWindow::mouseMoveEvent(QMouseEvent *e)
{
  // PRE CONDITION : mouseEvent, set_robot_flag, qnode->set_ball_flag
  // POST CONDITION : qnode->robot0.n, qnode->ball.n master_target_n,
  // purpose : set_object_flag가 활성화 되있고, 마우스 이동 이벤트가 적용 시
  // 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성
  QPointF scenePos = ui->screen->mapToScene(ui->screen->mapFrom(this, e->pos()));
  if (set_robot_flag)
  {
    qnode->mcl_.moveRobot(scenePos.x(), scenePos.y());
    qnode->clearMasterTarget();
  }
  if (!obstacle_mode && qnode->set_ball_flag)
  {
    qnode->tracker_.setBall(scenePos.x(), scenePos.y());
  }
}

QPolygonF MainWindow::create_Print_robot(const Pose2D &robot)
{
  // PRE CONDITION : robot.n, zoom(0.75)
  // POST CONDITION : Robot_model_poly
  // purpose : 로봇의 x, y, z좌표에 zoom값을 곱한 후 해당 값에 의한 폴리곤 생성

  float Robot_model_x = robot.x;
  float Robot_model_y = robot.y;
  float Robot_model_z = robot.yaw_deg + 90;

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
  // 현재 슬라이더 값을 로봇별 캘리브레이션 파일에 저장
  param_store::OdomParams p;
  p.particle_range = particle_range;
  p.odom_scale_forward = qnode->mcl_.motion.config.odom_scale_forward;
  p.odom_scale_backward = qnode->mcl_.motion.config.odom_scale_backward;
  p.odom_scale_left = qnode->mcl_.motion.config.odom_scale_left;
  p.odom_scale_right = qnode->mcl_.motion.config.odom_scale_right;
  param_store::save(p);
}

// 로봇 위치 설정 헬퍼 (좌표 확정 + 파티클 재살포 + 자취 초기화)
static void setRobotPos(QNode *qnode, float x, float y, float particle_range)
{
  qnode->mcl_.placeRobot(x, y, particle_range);
  qnode->clearMasterTarget();
}

void MainWindow::on_btn_set_1_clicked()
{
  const auto p = StartPositions::manualSlot(1, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_2_clicked()
{
  const auto p = StartPositions::manualSlot(2, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_3_clicked()
{
  const auto p = StartPositions::manualSlot(3, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_4_clicked()
{
  const auto p = StartPositions::manualSlot(4, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_5_clicked()
{
  const auto p = StartPositions::manualSlot(5, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_6_clicked()
{
  const auto p = StartPositions::manualSlot(6, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_set_auto_clicked()
{
  // get in 시 로봇 번호에 따라 자동 배치
  const auto p =
      StartPositions::autoByRobotNum(qnode->gameMSG.robotNum, qnode->gameMSG.mySide);
  setRobotPos(qnode, p.x, p.y, particle_range);
}

void MainWindow::on_btn_ball_set_clicked()
{
  if (obstacle_mode)
  {
    qnode->set_obstacle_flag = 1;
  }
  else
  {
    qnode->set_ball_flag = 1;
  }
}
void MainWindow::on_btn_ball_del_clicked()
{
  if (obstacle_mode)
  {
    qnode->tracker_.popDebugObstacle();
  }
  else
  {
    qnode->set_ball_flag = 0;
    qnode->tracker_.clearBall();
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








  qnode->resetTeammates();
  qnode->tracker_.clearTeammateBalls();
  qnode->mcl_.clearObservations();
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

MainWindow::~MainWindow()
{
  // 순서가 중요하다.
  //  1) 타이머를 먼저 멈춘다 — main()/Print_Screen() 이 qnode 를 계속 부르므로,
  //     qnode 를 먼저 지우면 파괴된 객체를 타이머가 건드린다
  //  2) qnode 는 부모 없는 QObject 라 직접 delete 해야 한다. 이걸 빠뜨리면
  //     ~QNode 가 안 돌아 ROS·MCL 스레드가 join 되지 않고, 프로세스 종료 시
  //     joinable 한 std::thread 소멸자가 terminate 를 부른다
  if (m_DataTimer)
    m_DataTimer->stop();
  if (m_screenTimer)
    m_screenTimer->stop();

  delete qnode;
  qnode = nullptr;

  delete ui;
}
