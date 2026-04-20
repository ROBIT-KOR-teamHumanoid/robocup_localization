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

#include "../include/robocup_localization/main_window.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <QMessageBox>
#include <QtGui>
#include <fstream>
#include <iostream>
#include <mutex>
#include <vector>

#include "robot_config.h"
#include "../include/field_coordinate/field_coordinate.hpp"
#include "../include/technical_challenge/technical_challenge.hpp"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode(argc, argv);

  // 데이터 업데이트 타이머
  m_DataTimer = new QTimer(this);
  connect(m_DataTimer, SIGNAL(timeout()), this, SLOT(main()));
  if (qnode->corner_kick_receiver_)
  {
    m_DataTimer->start(20);
  }
  else
  {
    m_DataTimer->start(50);
  }

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
  qnode->robot0.x = FieldCoord::MAP_W / 2; // 로봇 시작 포인트, 맵의 중앙
  qnode->robot0.y = FieldCoord::MAP_H / 2;

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::main()
{
  // INIT
  setting();     // 공 선택 & 파라미터 불러오기 & UI에서 오도메트리 값 설정
  select_ball(); // UDP 통신에서 받은 다른 로봇의 공데이터로 공 위치 설정 & obstacle 위치 계산

  if (qnode->corner_kick_receiver_ && qnode->gameMSG.state == 7 && qnode->gameMSG.position == 3)
  {
    qnode->technical_challengeMSG.ball_in_approach =
        technical_challenge::is_ball_in_approach_circle(
            qnode->robot0.x, qnode->robot0.y,
            qnode->ball.x, qnode->ball.y,
            qnode->robot0.z)
            ? 1
            : 0;
    qnode->publish_technical_challenge();
  }

  if (qnode->local_helper_)
  {
    qnode->publish_technical_challenge();
  }

  // 퍼블리시할 데이터를 localMSG에 채운 후 publish
  qnode->localMSG.ball_x = qnode->ball.x;
  qnode->localMSG.ball_y = qnode->ball.y;
  qnode->localMSG.ball_speed_x = qnode->ball.x + qnode->ball.speed_x * qnode->visionMSG.Ball_speed_level;
  qnode->localMSG.ball_speed_y = qnode->ball.y + qnode->ball.speed_y * qnode->visionMSG.Ball_speed_level;
  qnode->localMSG.robot_x = qnode->robot0.x;
  qnode->localMSG.robot_y = qnode->robot0.y;
  qnode->localMSG.obstacles_x.clear();
  qnode->localMSG.obstacles_y.clear();
  for (const auto &obs : qnode->obstacles)
  {
    qnode->localMSG.obstacles_x.push_back(obs.x);
    qnode->localMSG.obstacles_y.push_back(obs.y);
  }
  qnode->publish_localization();
}

void MainWindow::Print_Screen_Ball()
{
  if (!scene)
    return;
  QPen blackPen(Qt::black);
  QBrush redBrush(Qt::red);
  QBrush yellowBrush(Qt::yellow);
  QBrush blueBrush(Qt::blue);
  if (qnode->visionMSG.Ball_2d_X != 0 || qnode->visionMSG.Ball_2d_Y != 0)
  {
    scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                      cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen, redBrush);
    scene->addLine(cvt_Print_xy(qnode->ball.x), cvt_Print_xy(qnode->ball.y),
                   cvt_Print_xy(qnode->ball.x + qnode->ball.speed_x * qnode->visionMSG.Ball_speed_level),
                   cvt_Print_xy(qnode->ball.y + qnode->ball.speed_y * qnode->visionMSG.Ball_speed_level),
                   blackPen);
  }
  else if (qnode->ball.x != 0 || qnode->ball.y != 0)
  {
    scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                      cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen, yellowBrush);
  }
  if (qnode->set_ball_flag)
  {
    scene->addEllipse(cvt_Print_xy(qnode->ball.x) - 5,
                      cvt_Print_xy(qnode->ball.y) - 5, 11, 11, blackPen, blueBrush);
  }
}

void MainWindow::select_ball()
{
  // PRE CONDITION : robotX.b, qnode->visionMSG, ball 데이터 - 해당 데이터는
  // 다른 노드에서 통신받아 옴 POST CONDITION : obstacleX, ball 데이터 purpose :
  // 다른 로봇에서 받아 온 공 데이터로 최종 공 위치 설정, 이때 최종 공 위치는 각
  // 로봇의 공 데이터 중 가장 공과 가까운 로봇의 데이터로 설정 & obstacle 의
  // 위치 설정

  // 로봇의 공 데이터 초기화
  qnode->robot1.b -= 0.005;
  qnode->robot2.b -= 0.005;
  qnode->robot3.b -= 0.005;
  qnode->robot4.b -= 0.005;

  // 다른 로봇에서 공 데이터를 받아오지 않으면, 해당 로봇의 데이터 비활성화
  if (qnode->robot1.b <= 0)
  {
    qnode->robot1.x = 0;
    qnode->robot1.y = 0;
    qnode->robot1.z = 0;
    qnode->robot1.b = 0;
    qnode->robot1.state = 0;
    qnode->ball1.x = 0;
    qnode->ball1.y = 0;
    qnode->ball1.d = 999999;
  }
  if (qnode->robot2.b <= 0)
  {
    qnode->robot2.x = 0;
    qnode->robot2.y = 0;
    qnode->robot2.z = 0;
    qnode->robot2.b = 0;
    qnode->robot2.state = 0;
    qnode->ball2.x = 0;
    qnode->ball2.y = 0;
    qnode->ball2.d = 999999;
  }
  if (qnode->robot3.b <= 0)
  {
    qnode->robot3.x = 0;
    qnode->robot3.y = 0;
    qnode->robot3.z = 0;
    qnode->robot3.b = 0;
    qnode->robot3.state = 0;
    qnode->ball3.x = 0;
    qnode->ball3.y = 0;
    qnode->ball3.d = 999999;
  }
  if (qnode->robot4.b <= 0)
  {
    qnode->robot4.x = 0;
    qnode->robot4.y = 0;
    qnode->robot4.z = 0;
    qnode->robot4.b = 0;
    qnode->robot4.state = 0;
    qnode->ball4.x = 0;
    qnode->ball4.y = 0;
    qnode->ball4.d = 999999;
  }

  // ROBOT_VEC 크기에 맞게 obstacles 벡터 크기 조정
  size_t vec_size = qnode->visionMSG.ROBOT_VEC_X.size();
  if (qnode->obstacles.size() < vec_size)
  {
    qnode->obstacles.resize(vec_size);
  }

  double yaw_rad = qnode->robot0.z * M_PI / 180;
  for (size_t i = 0; i < qnode->obstacles.size(); i++)
  {
    if (i < vec_size)
    {
      qnode->obstacles[i].x =
          qnode->robot0.x + (-1) * (qnode->visionMSG.ROBOT_VEC_X[i] * FieldCoord::MM2PX *
                                        cos(yaw_rad) +
                                    qnode->visionMSG.ROBOT_VEC_Y[i] * FieldCoord::MM2PX *
                                        sin(yaw_rad));
      qnode->obstacles[i].y =
          qnode->robot0.y + (qnode->visionMSG.ROBOT_VEC_X[i] * FieldCoord::MM2PX *
                                 sin(yaw_rad) -
                             qnode->visionMSG.ROBOT_VEC_Y[i] * FieldCoord::MM2PX *
                                 cos(yaw_rad));
      qnode->obstacles[i].nocnt = 0;
    }
    else
    {
      qnode->obstacles[i].nocnt += 1;
      if (qnode->obstacles[i].nocnt > 20)
      {
        qnode->obstacles[i].nocnt = 20;
        qnode->obstacles[i].x = 0;
        qnode->obstacles[i].y = 0;
      }
    }
  }

  // qnode->visionMSG.Ball_2d 의 데이터를 통해 비전에서 받아온 공 데이터를 로컬
  // 좌표로 변환
  if (qnode->visionMSG.Ball_2d_X != 0 || qnode->visionMSG.Ball_2d_Y != 0)
  {
    qnode->ball.noballcnt = 0;
    qnode->ball.x =
        qnode->robot0.x + (-1) * (qnode->visionMSG.Ball_2d_X * FieldCoord::MM2PX *
                                      cos((qnode->robot0.z) * M_PI / 180) +
                                  qnode->visionMSG.Ball_2d_Y * FieldCoord::MM2PX *
                                      sin((qnode->robot0.z) * M_PI / 180));
    qnode->ball.y =
        qnode->robot0.y +
        (qnode->visionMSG.Ball_2d_X * FieldCoord::MM2PX * sin((qnode->robot0.z) * M_PI / 180) -
         qnode->visionMSG.Ball_2d_Y * FieldCoord::MM2PX * cos((qnode->robot0.z) * M_PI / 180));
    qnode->ball.speed_x =
        (qnode->visionMSG.Ball_speed_X) *
            cos((-1) * qnode->robot0.z * DEG2RAD) -
        (qnode->visionMSG.Ball_speed_Y) * sin((-1) * qnode->robot0.z * DEG2RAD);
    qnode->ball.speed_y =
        (qnode->visionMSG.Ball_speed_X) *
            sin((-1) * qnode->robot0.z * DEG2RAD) +
        (qnode->visionMSG.Ball_speed_Y) * cos((-1) * qnode->robot0.z * DEG2RAD);
  }
  // 데이터를 받지 않으면 데이터 비활성화
  else
  {
    qnode->ball.noballcnt += 1;
    if (qnode->ball.noballcnt >= 300) // 약 15초
    {
      qnode->ball.x = 0;
      qnode->ball.y = 0;
      qnode->ball.noballcnt = 300;
    }
  }

  double min_d = qnode->visionMSG.Ball_D /
                 10; // 공 최종위치 판별 시 공 위치를 판별할 예외처리 변수 선언
  if (qnode->visionMSG.Ball_D == 0)
  {
    min_d = 999999;
  }
  int min_i = 0;

  // 각 로봇에서 측정된 공 데이터를 비교하여 해당 공 데이터가 최소값일
  // 경우(측정된 로봇과 가장 가까울 경우) 해당 값 적용
  if (min_d > qnode->ball1.d)
  {
    min_d = qnode->ball1.d;
    min_i = 1;
  }
  if (min_d > qnode->ball2.d)
  {
    min_d = qnode->ball2.d;
    min_i = 2;
  }
  if (min_d > qnode->ball3.d)
  {
    min_d = qnode->ball3.d;
    min_i = 3;
  }
  if (min_d > qnode->ball4.d)
  {
    min_d = qnode->ball4.d;
    min_i = 4;
  }

  // 최소값인 공 데이터가 해당 로봇이 아닐때 최소값인 공 데이터 적용
  if (min_i != 0 &&
      (qnode->ball.noballcnt == 0 || qnode->ball.noballcnt == 100))
  {
    if (min_i == 1)
    {
      qnode->ball.x = qnode->ball1.x;
      qnode->ball.y = qnode->ball1.y;
    }
    if (min_i == 2)
    {
      qnode->ball.x = qnode->ball2.x;
      qnode->ball.y = qnode->ball2.y;
    }
    if (min_i == 3)
    {
      qnode->ball.x = qnode->ball3.x;
      qnode->ball.y = qnode->ball3.y;
    }
    if (min_i == 4)
    {
      qnode->ball.x = qnode->ball4.x;
      qnode->ball.y = qnode->ball4.y;
    }
    qnode->ball.noballcnt = 25;
  }

  // 공/장애물 좌표 클램핑 (필드 경계 픽셀 기준)
  auto clampX = [](auto &v)
  {
    if (v < FieldCoord::F_X_MIN && v != 0)
      v = FieldCoord::F_X_MIN;
    if (v > FieldCoord::F_X_MAX)
      v = FieldCoord::F_X_MAX;
  };
  auto clampY = [](auto &v)
  {
    if (v < FieldCoord::F_Y_MIN && v != 0)
      v = FieldCoord::F_Y_MIN;
    if (v > FieldCoord::F_Y_MAX)
      v = FieldCoord::F_Y_MAX;
  };

  clampX(qnode->ball.x);
  clampY(qnode->ball.y);
  for (auto &obs : qnode->obstacles)
  {
    clampX(obs.x);
    clampY(obs.y);
  }
}

void MainWindow::setting()
{
  // PRE CONDITION : qnode->vision_callback_timer, qnode->set_ball_flag,
  // ball_set_N, setting_flag, 파라미터 저장파일 POST CONDITION : ball_N,
  // qnode->robot0.odom_Nn purpose : 로봇의 초기 세팅 및 입출력스트림을 통해
  // 저장되어있던 오도메트리 값 설정, 볼의 위치를 재설정 하는 경우 볼 위치 변경,
  // ui슬라이더를 통해 오도메트리 값 적용

  qnode->vision_callback_timer += 1; // vision timer count +1
  if (qnode->set_ball_flag)          // qnode->set_ball_flag가 1일때 공 위치 변경
  {
    qnode->ball.x = qnode->ball.set_x;
    qnode->ball.y = qnode->ball.set_y;
    qnode->ball.noballcnt = 0;
  }
  if (qnode->vision_callback_timer > 10 &&
      qnode->set_ball_flag == 0) // qnode->set_ball_flag가 비활성화 상태이고
                                 // timer가 10 이상이면 공 위치 비활성화
  {
    qnode->ball.x = 0;
    qnode->ball.y = 0;
  }

  if (setting_flag ==
      0) // setting_flag가 0이면 입출력스트림을 통해 데이터 로드
  {
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
    ifstream Last_Index_Num_IN(param);
    if (Last_Index_Num_IN.is_open())
    {
      Last_Index_Num_IN >> particle_range;
      Last_Index_Num_IN >> qnode->robot0.odom_fx;
      Last_Index_Num_IN >> qnode->robot0.odom_bx;
      Last_Index_Num_IN >> qnode->robot0.odom_ly;
      Last_Index_Num_IN >> qnode->robot0.odom_ry;
      printf("[LocalNode] Odom Load\n");
    }
    else
    {
      printf("[LocalNode] No Odom Data\n");
    }
    Last_Index_Num_IN.close(); // setting_flag가 0이면 로드 된 데이터를 ui슬라이더에 적용
    ui->particle_range_value->setValue(particle_range);
    ui->odom_fx_value->setValue(qnode->robot0.odom_fx);
    ui->odom_bx_value->setValue(qnode->robot0.odom_bx);
    ui->odom_ly_value->setValue(qnode->robot0.odom_ly);
    ui->odom_ry_value->setValue(qnode->robot0.odom_ry);
    setting_flag = 1; // setting_flag set 1
  }
  // ui의 슬라이더 값을 오도메트리 값에 적용
  particle_range = ui->particle_range_value->value();
  qnode->particle_range = particle_range;
  qnode->robot0.odom_fx = ui->odom_fx_value->value();
  qnode->robot0.odom_bx = ui->odom_bx_value->value();
  qnode->robot0.odom_ly = ui->odom_ly_value->value();
  qnode->robot0.odom_ry = ui->odom_ry_value->value();
}

void MainWindow::Print_Screen() // ui 출력, 200ms마다 실행, 공 출력은 50ms로 분리
{
  // PRE CONDITION : robotX.n, obstacleX.n, qnode->ball.n, qnode->Likelihood
  // POST CONDITION : UI
  // purpose : UI출력

  // INIT
  if (scene)
  {
    delete scene;
  }

  char TEXT[256];
  sprintf(TEXT, "BALL_X : %d", (int)qnode->localMSG.ball_x);
  ui->BALL_X_value->setText(TEXT);
  sprintf(TEXT, "BALL_Y : %d", (int)qnode->localMSG.ball_y);
  ui->BALL_Y_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_X : %d", (int)qnode->localMSG.robot_x);
  ui->ROBOT_X_value->setText(TEXT);
  sprintf(TEXT, "ROBOT_Y : %d", (int)qnode->localMSG.robot_y);
  ui->ROBOT_Y_value->setText(TEXT);
  sprintf(TEXT, "IK_X : %d", (int)qnode->robot0.now_ik_param_x);
  ui->IK_X_value->setText(TEXT);
  sprintf(TEXT, "IK_Y : %d", (int)qnode->robot0.now_ik_param_y);
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
  for (const auto &obs : qnode->obstacles)
  {
    if (obs.x != 0 && obs.y != 0)
    {
      scene->addRect(cvt_Print_xy(obs.x) - 5,
                     cvt_Print_xy(obs.y) - 5, 10, 10, blackPen,
                     blackBrush);
    }
  }

  // PRINT PARTICLE
  if (true)
  {
    for (int i = 0; i < qnode->particle_num; i++)
    {
      scene->addLine(cvt_Print_xy(qnode->pt[i].x), cvt_Print_xy(qnode->pt[i].y),
                     cvt_Print_xy(qnode->pt[i].x), cvt_Print_xy(qnode->pt[i].y),
                     grayPen);
    }
  }

  // PRINT TIMESTAMP
  if (qnode->robot0.TIME_STAMP.size() > 0)
  {
    scene->addLine(
        cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
        cvt_Print_xy(
            qnode->robot0.TIME_STAMP[qnode->robot0.TIME_STAMP.size() - 1].X),
        cvt_Print_xy(
            qnode->robot0.TIME_STAMP[qnode->robot0.TIME_STAMP.size() - 1].Y),
        blackPen3);
    for (int i = 1; i < qnode->robot0.TIME_STAMP.size(); i++)
    {
      scene->addLine(cvt_Print_xy(qnode->robot0.TIME_STAMP[i - 1].X),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i - 1].Y),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i].X),
                     cvt_Print_xy(qnode->robot0.TIME_STAMP[i].Y), blackPen3);
    }
  }

  // PRINT SIGHT
  // if(qnode->visionMSG.Scan_mode != 3){qnode->robot_sight_flag = 0;
  // qnode->Likelihood.vision_point_vect.clear();}

  // 비전에서 측정된 특징점 표시
  for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
  {
    if (!FieldCoord::ALL_POINTS[i].active)
      continue;
    if (qnode->Likelihood.filter.Local_point_on_off[i] == 1)
    {
      scene->addEllipse(cvt_Print_xy(qnode->Likelihood.filter.Local_point_x[i]) - 4,
                        cvt_Print_xy(qnode->Likelihood.filter.Local_point_y[i]) - 4, 8,
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

    for (int i = 0; i < (int)qnode->Likelihood.vision_point_vect.size(); i++)
    {
      auto &vp = qnode->Likelihood.vision_point_vect[i];

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
        double dis = sqrt(pow(qnode->Likelihood.filter.Local_point_x[j] - obs_px, 2) +
                          pow(qnode->Likelihood.filter.Local_point_y[j] - obs_py, 2));
        if (dis < min_dis)
        {
          min_dis = dis;
          nearest_j = j;
        }
      }
      if (nearest_j >= 0)
      {
        scene->addEllipse(cvt_Print_xy(qnode->Likelihood.filter.Local_point_x[nearest_j]) - 6,
                          cvt_Print_xy(qnode->Likelihood.filter.Local_point_y[nearest_j]) - 6,
                          12, 12, nearbyPen, nearbyBrush);
      }
    }
  }

  
  for (int i = 0; i < qnode->Likelihood.vision_point_vect.size(); i++)
  {
    auto px = cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_X +
                           qnode->Likelihood.vision_point_vect[i].POINT_VEC_X);
    auto py = cvt_Print_xy(qnode->Likelihood.vision_point_vect[i].STD_Y +
                           qnode->Likelihood.vision_point_vect[i].POINT_VEC_Y);
    if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.9)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen1);
    }
    else if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.7)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen2);
    }
    else if (qnode->Likelihood.vision_point_vect[i].CONFIDENCE > 0.5)
    {
      scene->addEllipse(px - 3, py - 3, 6, 6, LocalPen3);
    }
  }
  if (qnode->Likelihood.vision_point_vect.size() != 0)
  {
   
    scene->addEllipse(cvt_Print_xy(qnode->Likelihood.filter.CIRCLE_CENTER.x -
                                   qnode->Likelihood.filter.CIRCLE_R),
                      cvt_Print_xy(qnode->Likelihood.filter.CIRCLE_CENTER.y -
                                   qnode->Likelihood.filter.CIRCLE_R),
                      cvt_Print_xy(qnode->Likelihood.filter.CIRCLE_R * 2),
                      cvt_Print_xy(qnode->Likelihood.filter.CIRCLE_R * 2), blackPen);
  }

  // PRINT MASTER TARGET
  if (qnode->master_target_x != 0 || qnode->master_target_y != 0)
  {
    scene->addLine(cvt_Print_xy(qnode->master_target_x) + 4,
                   cvt_Print_xy(qnode->master_target_y),
                   cvt_Print_xy(qnode->master_target_x) - 4,
                   cvt_Print_xy(qnode->master_target_y), targetPen);
    scene->addLine(cvt_Print_xy(qnode->master_target_x),
                   cvt_Print_xy(qnode->master_target_y) + 4,
                   cvt_Print_xy(qnode->master_target_x),
                   cvt_Print_xy(qnode->master_target_y) - 4, targetPen);
    scene->addLine(cvt_Print_xy(qnode->master_target_x) + 2,
                   cvt_Print_xy(qnode->master_target_y) + 2,
                   cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
                   targetlinePen);
  }

  // PRINT ROBOT
  QPolygonF robot0_poly = create_Print_robot(qnode->robot0);
  if (qnode->robot1.state != 3 && qnode->robot2.state != 3 &&
      qnode->robot3.state != 3 && qnode->robot4.state != 3 &&
      (qnode->ball.x != 0 || qnode->ball.y != 0))
  {
    scene->addPolygon(robot0_poly, redPen3, blueBrush);
  }
  else
  {
    scene->addPolygon(robot0_poly, blackPen, blueBrush);
  }
  scene->addLine(
      cvt_Print_xy(qnode->robot0.x), cvt_Print_xy(qnode->robot0.y),
      cvt_Print_xy(qnode->robot0.x) +
          sin((-1) * qnode->robot0.z * DEG2RAD) * 10,
      cvt_Print_xy(qnode->robot0.y) - cos(qnode->robot0.z * DEG2RAD) * 10,
      blackPen3);

  if (qnode->corner_kick_receiver_ && qnode->gameMSG.state == 7 && qnode->gameMSG.position == 3)
  {
    technical_challenge::draw_approach_circle(
        scene,
        cvt_Print_xy(qnode->robot0.x),
        cvt_Print_xy(qnode->robot0.y),
        qnode->robot0.z);
  }

  if (qnode->robot1.x != 0 || qnode->robot1.y != 0)
  {
    QPolygonF robot1_poly = create_Print_robot(qnode->robot1);

    if (qnode->robot1.state == 3)
    {
      scene->addPolygon(robot1_poly, redPen3, redBrush);
    }
    else
    {
      scene->addPolygon(robot1_poly, blackPen, redBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot1.x), cvt_Print_xy(qnode->robot1.y),
        cvt_Print_xy(qnode->robot1.x) +
            sin((-1) * qnode->robot1.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot1.y) - cos(qnode->robot1.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot2.x != 0 || qnode->robot2.y != 0)
  {
    QPolygonF robot2_poly = create_Print_robot(qnode->robot2);
    if (qnode->robot2.state == 3)
    {
      scene->addPolygon(robot2_poly, redPen3, greenBrush);
    }
    else
    {
      scene->addPolygon(robot2_poly, blackPen, greenBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot2.x), cvt_Print_xy(qnode->robot2.y),
        cvt_Print_xy(qnode->robot2.x) +
            sin((-1) * qnode->robot2.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot2.y) - cos(qnode->robot2.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot3.x != 0 || qnode->robot3.y != 0)
  {
    QPolygonF robot3_poly = create_Print_robot(qnode->robot3);
    if (qnode->robot3.state == 3)
    {
      scene->addPolygon(robot3_poly, redPen3, yellowBrush);
    }
    else
    {
      scene->addPolygon(robot3_poly, blackPen, yellowBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot3.x), cvt_Print_xy(qnode->robot3.y),
        cvt_Print_xy(qnode->robot3.x) +
            sin((-1) * qnode->robot3.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot3.y) - cos(qnode->robot3.z * DEG2RAD) * 10,
        blackPen3);
  }
  if (qnode->robot4.x != 0 || qnode->robot4.y != 0)
  {
    QPolygonF robot4_poly = create_Print_robot(qnode->robot4);
    if (qnode->robot4.state == 3)
    {
      scene->addPolygon(robot4_poly, redPen3, blackBrush);
    }
    else
    {
      scene->addPolygon(robot4_poly, blackPen, blackBrush);
    }
    scene->addLine(
        cvt_Print_xy(qnode->robot4.x), cvt_Print_xy(qnode->robot4.y),
        cvt_Print_xy(qnode->robot4.x) +
            sin((-1) * qnode->robot4.z * DEG2RAD) * 10,
        cvt_Print_xy(qnode->robot4.y) - cos(qnode->robot4.z * DEG2RAD) * 10,
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
    qnode->robot0.x = scenePos.x();
    qnode->robot0.y = scenePos.y();
    {
      std::lock_guard<std::mutex> pt_lock(qnode->pt_mutex);
      for (int i = 0; i < qnode->particle_num; i++)
      {
        qnode->Likelihood.random_point(qnode->pt[i], qnode->robot0.x, qnode->robot0.y,
                                       particle_range, qnode->robot0.z);
      }
    }
    set_robot_flag = 0;
    qnode->robot0.TIME_STAMP.clear();
    qnode->master_target_x = 0;
    qnode->master_target_y = 0;
  }

  if (qnode->set_ball_flag)
  {
    qnode->ball.x = scenePos.x();
    qnode->ball.y = scenePos.y();
    qnode->ball.set_x = qnode->ball.x;
    qnode->ball.set_y = qnode->ball.y;
    qnode->ball.noballcnt = 0;
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
    qnode->robot0.x = scenePos.x();
    qnode->robot0.y = scenePos.y();
    qnode->robot0.TIME_STAMP.clear();
    qnode->master_target_x = 0;
    qnode->master_target_y = 0;
  }
  if (qnode->set_ball_flag)
  {
    qnode->ball.x = scenePos.x();
    qnode->ball.y = scenePos.y();
    qnode->ball.set_x = qnode->ball.x;
    qnode->ball.set_y = qnode->ball.y;
    qnode->ball.noballcnt = 0;
  }
}

QPolygonF MainWindow::create_Print_robot(ROBOT robot)
{
  // PRE CONDITION : robot.n, zoom(0.75)
  // POST CONDITION : Robot_model_poly
  // purpose : 로봇의 x, y, z좌표에 zoom값을 곱한 후 해당 값에 의한 폴리곤 생성

  float Robot_model_x = robot.x;
  float Robot_model_y = robot.y;
  float Robot_model_z = robot.z + 90;

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

void MainWindow::featureCalc()
{
  qnode->Likelihood.featureCalc(qnode, particle_range);
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
    Last_Index_Num_OUT << qnode->robot0.odom_fx << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_bx << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_ly << endl;
    Last_Index_Num_OUT << qnode->robot0.odom_ry << endl;
    printf("[LocalNode] Odom Param set\n");
  }
  else
  {
    printf("[LocalNode] No Odom Parameter\n");
  }
  Last_Index_Num_OUT.close();
}

// 로봇 위치 설정 헬퍼 (좌표 설정 + 파티클 재생성 + 타임스탬프 초기화)
static void setRobotPos(QNode *qnode, float x, float y, float particle_range)
{
  qnode->robot0.x = x;
  qnode->robot0.y = y;
  {
    std::lock_guard<std::mutex> pt_lock(qnode->pt_mutex);
    for (int i = 0; i < qnode->particle_num; i++)
      qnode->Likelihood.random_point(qnode->pt[i], qnode->robot0.x, qnode->robot0.y, particle_range, qnode->robot0.z);
  }
  qnode->robot0.TIME_STAMP.clear();
  qnode->master_target_x = 0;
  qnode->master_target_y = 0;
}

void MainWindow::on_btn_set_1_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(0.8), toPixelY(B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-3.5), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_2_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(2.5), toPixelY(B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-2.5), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_3_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(3.5), toPixelY(B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-0.8), toPixelY(B / 2), particle_range);
}
void MainWindow::on_btn_set_4_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(0.8), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-3.5), toPixelY(-B / 2), particle_range);
}
void MainWindow::on_btn_set_5_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(2.5), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-2.5), toPixelY(-B / 2), particle_range);
}
void MainWindow::on_btn_set_6_clicked()
{
  using namespace FieldCoord;
  if (qnode->gameMSG.mySide)
    setRobotPos(qnode, toPixelX(3.5), toPixelY(-B / 2), particle_range);
  else
    setRobotPos(qnode, toPixelX(-0.8), toPixelY(-B / 2), particle_range);
}

void MainWindow::on_btn_set_7_clicked()
{
  using namespace FieldCoord;
  qnode->robot0.z = -90.0;
  qnode->local_helper_dir = 0;
  setRobotPos(qnode, toPixelX(-A / 2), toPixelY(F / 2 + (H - F) / 4), 0);
}

void MainWindow::on_btn_set_8_clicked()
{
  using namespace FieldCoord;
  qnode->robot0.z = 90.0;
  qnode->local_helper_dir = 1;
  setRobotPos(qnode, toPixelX(A / 2), toPixelY(-F / 2 - (H - F) / 4), 0);
}

void MainWindow::on_btn_set_auto_clicked()
{
  // PRE CONDITION : UI
  // POST CONDITION : qnode->robot0.N, qnode->master_target_x
  // purpose : get in시 위치 자동 매크로
  using namespace FieldCoord;

  float x = 0, y = 0;
  if (qnode->gameMSG.mySide)
  {
    if (qnode->gameMSG.robotNum == 1)
    {
      x = toPixelX(0.8);
      y = toPixelY(B / 2);
    }
    else if (qnode->gameMSG.robotNum == 2)
    {
      x = toPixelX(2.5);
      y = toPixelY(B / 2);
    }
    else if (qnode->gameMSG.robotNum == 3)
    {
      x = toPixelX(0.8);
      y = toPixelY(-B / 2);
    }
    else if (qnode->gameMSG.robotNum == 4)
    {
      x = toPixelX(3.5);
      y = toPixelY(-B / 2);
    }
  }
  else
  {
    if (qnode->gameMSG.robotNum == 1)
    {
      x = toPixelX(-0.8);
      y = toPixelY(-B / 2);
    }
    else if (qnode->gameMSG.robotNum == 2)
    {
      x = toPixelX(-2.5);
      y = toPixelY(-B / 2);
    }
    else if (qnode->gameMSG.robotNum == 3)
    {
      x = toPixelX(-0.8);
      y = toPixelY(B / 2);
    }
    else if (qnode->gameMSG.robotNum == 4)
    {
      x = toPixelX(-3.5);
      y = toPixelY(B / 2);
    }
  }
  setRobotPos(qnode, x, y, particle_range);
}
void MainWindow::on_btn_ball_set_clicked()
{
  // PRE CONDITION : UI
  // POST CONDITION : qnode->set_ball_flag
  // purpose : 공 설정

  qnode->set_ball_flag = 1;
}
void MainWindow::on_btn_ball_del_clicked()
{
  // PRE CONDITION : UI
  // POST CONDITION : qnode->set_ball_flag, qnode->ball.N
  // purpose : 공 삭제 버튼

  qnode->set_ball_flag = 0;
  qnode->ball.x = 0;
  qnode->ball.y = 0;
}
void MainWindow::on_btn_test_clicked()
{
  qnode->robot1.x = 0;
  qnode->robot1.y = 0;
  qnode->robot1.z = 0;

  qnode->ball1.x = 0;
  qnode->ball1.y = 0;
  qnode->ball1.d = 999999;

  qnode->robot2.x = 0;
  qnode->robot2.y = 0;
  qnode->robot2.z = 0;

  qnode->ball2.x = 0;
  qnode->ball2.y = 0;
  qnode->ball2.d = 999999;

  qnode->robot3.x = 0;
  qnode->robot3.y = 0;
  qnode->robot3.z = 0;

  qnode->ball3.x = 0;
  qnode->ball3.y = 0;
  qnode->ball3.d = 999999;

  qnode->robot4.x = 0;
  qnode->robot4.y = 0;
  qnode->robot4.z = 0;

  qnode->ball4.x = 0;
  qnode->ball4.y = 0;
  qnode->ball4.d = 999999;

  qnode->Likelihood.vision_point_vect.clear();
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
