/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robocup_localization/qnode.hpp"

#include "../../robot_config.h"
#include "../include/field_coordinate/field_coordinate.hpp"

/*****************************************************************************
** ROBOT class implementations
*****************************************************************************/

ROBOT::ROBOT() {}

ROBOT::ROBOT(float x, float y)
{
  this->x = x;
  this->y = y;
}

void ROBOT::create_time_stamp(double Xmoved, double Ymoved)
{
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : TIME_STAMP
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 이동 경로를 저장

  if (Xmoved != 0 || Ymoved != 0)
  {
    time_t timer;
    timer = time(NULL);
    point.TIME = timer;
    point.X = this->x;
    point.Y = this->y;
    TIME_STAMP.push_back(point);
    if (TIME_STAMP.size() > 10)
    {
      TIME_STAMP.erase(TIME_STAMP.begin());
    }
  }
  else
  {
    TIME_STAMP.clear();
  }
}

void ROBOT::move(double Xmoved, double Ymoved)
{
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : x, y
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 해당 값으로 이동

  create_time_stamp(Xmoved, Ymoved);

  this->now_ik_param_x = Xmoved;
  this->now_ik_param_y = Ymoved;
  if (Xmoved > 0)
    Xmoved *= odom_fx;
  else
    Xmoved *= odom_bx;

  if (Ymoved > 0)
    Ymoved *= odom_ly;
  else
    Ymoved *= odom_ry;

  this->x += Xmoved * cos((this->z + 90) * DEG2RAD) - Ymoved * sin((this->z + 90) * DEG2RAD);
  this->y -= Xmoved * sin((this->z + 90) * DEG2RAD) + Ymoved * cos((this->z + 90) * DEG2RAD);

  if (this->x < FieldCoord::toPixelX(-FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(-FieldCoord::A / 2);
  if (this->x > FieldCoord::toPixelX(FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(FieldCoord::A / 2);
  if (this->y < FieldCoord::toPixelY(FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(FieldCoord::B / 2);
  if (this->y > FieldCoord::toPixelY(-FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(-FieldCoord::B / 2);
}


/*****************************************************************************
** QNode
*****************************************************************************/

QNode::QNode(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robocup_localization");
  this->start();

  node->declare_parameter("robot_feature", "kids");
  node->declare_parameter("topic.imu_sub_topic", "Imu");
  node->declare_parameter("topic.vision_sub_topic", "vision");
  node->declare_parameter("topic.vision_feature_sub_topic", "vision_feature");
  node->declare_parameter("topic.ik_sub_topic", "ikcoordinate");
  node->declare_parameter("topic.game_control_sub_topic", "gamecontroldata");
  node->declare_parameter("topic.udp_sub_topic", "udp");
  node->declare_parameter("topic.master_sub_topic", "master2local");
  node->declare_parameter("topic.local_pub_topic", "localization");
  node->declare_parameter("particle_filter.weight_update_threshold", 50.0);
  node->declare_parameter("particle_filter.particle_nums", particle_num);
  auto imu_topic = node->get_parameter("topic.imu_sub_topic").as_string();
  auto vision_topic = node->get_parameter("topic.vision_sub_topic").as_string();
  auto vision_feature_topic = node->get_parameter("topic.vision_feature_sub_topic").as_string();
  auto ik_topic = node->get_parameter("topic.ik_sub_topic").as_string();
  auto game_control_topic = node->get_parameter("topic.game_control_sub_topic").as_string();
  auto udp_topic = node->get_parameter("topic.udp_sub_topic").as_string();
  auto master_topic = node->get_parameter("topic.master_sub_topic").as_string();
  auto local_pub_topic = node->get_parameter("topic.local_pub_topic").as_string();

  imu_sub_ = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().best_effort(), std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  vision_sub_ = node->create_subscription<robocup_vision::msg::Robocupvision>(vision_topic, 1, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<robocup_vision::msg::Robocupvisionfeature>(vision_feature_topic, 4, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(ik_topic, 1, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<gamecontroller::msg::Gamecontroldata>(game_control_topic, 5, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>(udp_topic, 5, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization>(master_topic, 5, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<robocup_localization::msg::RobocupLocalization>(local_pub_topic, 5);

  // adult / kids different config
  robot_feature = node->get_parameter("robot_feature").as_string();

  weight_update_threshold = node->get_parameter("particle_filter.weight_update_threshold").as_double();
  particle_num = node->get_parameter("particle_filter.particle_nums").as_int();
  pt.resize(particle_num);

  if (robot_feature == "adult")
  {
    // 알아서 사용
  }
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::visionFeatureCallback(const robocup_vision::msg::Robocupvisionfeature::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(vision_mutex);

  // robot0 스냅샷 (robot0_mutex 보호)
  float robot0_x, robot0_y, robot0_z;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    robot0_x = robot0.x;
    robot0_y = robot0.y;
    robot0_z = robot0.z;
  }

  // L/ㄱ (corner)
  for (int i = 0; i < (int)msg->corner_confidence.size(); i++)
  {
    vision_data_cnt += 1;
    Likelihood.vision_point.CONFIDENCE   = msg->corner_confidence[i];
    Likelihood.vision_point.DISTANCE     = msg->corner_distance[i];
    Likelihood.vision_point.POINT_VEC_X  = (-1) * (msg->corner_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z) * M_PI / 180) + msg->corner_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z) * M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y  = (msg->corner_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z) * M_PI / 180) - msg->corner_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z) * M_PI / 180));
    Likelihood.vision_point.STD_X        = robot0_x;
    Likelihood.vision_point.STD_Y        = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 0;  // L
    if (i < (int)msg->corner_ball_relative_angle.size())
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = msg->corner_ball_relative_angle[i];
    else
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = 0.0;
    if (i < (int)msg->corner_ball_feature_distance.size())
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = msg->corner_ball_feature_distance[i];
    else
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = 0.0;
    Likelihood.vision_point_vect.push_back(Likelihood.vision_point);
  }
  vision_data_size += msg->corner_confidence.size();

  // T
  for (int i = 0; i < (int)msg->t_confidence.size(); i++)
  {
    vision_data_cnt += 1;
    Likelihood.vision_point.CONFIDENCE   = msg->t_confidence[i];
    Likelihood.vision_point.DISTANCE     = msg->t_distance[i];
    Likelihood.vision_point.POINT_VEC_X  = (-1) * (msg->t_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z) * M_PI / 180) + msg->t_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z) * M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y  = (msg->t_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z) * M_PI / 180) - msg->t_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z) * M_PI / 180));
    Likelihood.vision_point.STD_X        = robot0_x;
    Likelihood.vision_point.STD_Y        = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 1;  // T
    if (i < (int)msg->t_ball_relative_angle.size())
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = msg->t_ball_relative_angle[i];
    else
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = 0.0;
    if (i < (int)msg->t_ball_feature_distance.size())
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = msg->t_ball_feature_distance[i];
    else
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = 0.0;
    Likelihood.vision_point_vect.push_back(Likelihood.vision_point);
  }
  vision_data_size += msg->t_confidence.size();

  // X (cross)
  for (int i = 0; i < (int)msg->cross_confidence.size(); i++)
  {
    vision_data_cnt += 1;
    Likelihood.vision_point.CONFIDENCE   = msg->cross_confidence[i];
    Likelihood.vision_point.DISTANCE     = msg->cross_distance[i];
    Likelihood.vision_point.POINT_VEC_X  = (-1) * (msg->cross_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z) * M_PI / 180) + msg->cross_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z) * M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y  = (msg->cross_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z) * M_PI / 180) - msg->cross_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z) * M_PI / 180));
    Likelihood.vision_point.STD_X        = robot0_x;
    Likelihood.vision_point.STD_Y        = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 2;  // X

    
    if (i < (int)msg->cross_ball_relative_angle.size())
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = msg->cross_ball_relative_angle[i];
    else
      Likelihood.vision_point.BALL_RELATIVE_ANGLE = 0.0;
    if (i < (int)msg->cross_ball_feature_distance.size())
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = msg->cross_ball_feature_distance[i];
    else
      Likelihood.vision_point.BALL_FEATURE_DISTANCE = 0.0;
    Likelihood.vision_point_vect.push_back(Likelihood.vision_point);
  }
  vision_data_size += msg->cross_confidence.size();

  bool has_real_features = !msg->corner_confidence.empty() ||
                           !msg->t_confidence.empty()      ||
                           !msg->cross_confidence.empty();

  if (has_real_features && !Likelihood.vision_point_vect.empty() && visionMSG.flag_pan == 0)
  {
    // RCLCPP_INFO(node->get_logger(),
    //   "[OBS] vect=%zu (L=%d T=%d X=%d) total_cnt=%d",
    //   Likelihood.vision_point_vect.size(),
    //   (int)msg->corner_confidence.size(),
    //   (int)msg->t_confidence.size(),
    //   (int)msg->cross_confidence.size(),
    //   vision_data_cnt);
    Likelihood.filter.setCircle(robot0_z, Likelihood.vision_point_vect);
    emit featureReceived();
  }
}

void QNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(robot0_mutex);
  robot0.z = msg->yaw;
}

void QNode::visionCallback(const robocup_vision::msg::Robocupvision::SharedPtr msg)
{
  // PRE CONDITION : msg(vision)
  // POST CONDITION : Ball_cam_N, Ball_2d_N, Ball_D, PAN, TILT, Ball_speed_N, ROBOT_VEC_N, Ball_speed_level
  // purpose : 비전에서 받아온 데이터 콜백

  visionMSG.Ball_cam_X = msg->ball_cam_x;
  visionMSG.Ball_cam_Y = msg->ball_cam_y;
  visionMSG.Ball_2d_X = msg->ball_2d_x;
  visionMSG.Ball_2d_Y = msg->ball_2d_y;
  visionMSG.Ball_D = msg->ball_d;
  visionMSG.PAN = msg->pan;
  visionMSG.TILT = msg->tilt;
  visionMSG.Ball_speed_X = msg->ball_speed_x;
  visionMSG.Ball_speed_Y = msg->ball_speed_y;
  visionMSG.ROBOT_VEC_X.clear();
  visionMSG.ROBOT_VEC_Y.clear();
  for (int i = 0; i < msg->robot_vec_x.size(); i++)
  {
    visionMSG.ROBOT_VEC_X.push_back(msg->robot_vec_x[i]);
    visionMSG.ROBOT_VEC_Y.push_back(msg->robot_vec_y[i]);
  }
  visionMSG.Ball_speed_level = msg->ball_speed_level;
  visionMSG.Scan_mode = msg->scan_mode;
  visionMSG.flag_pan = msg->flag_pan;
  if (visionMSG.Ball_speed_level > 150)
  {
    visionMSG.Ball_speed_level = 150;
  }
  if (visionMSG.Ball_cam_X != 0 || visionMSG.Ball_cam_Y != 0)
  {
    set_ball_flag = 0;
  }
  vision_callback_timer = 0;
}

void QNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  // PRE CONDITION : msg(IK)
  // POST CONDITION : robot0 위치 업데이트, 파티클 예측 단계 적용
  // PURPOSE : MCL 예측(predict) 단계
  //   robot0.move()로 오도메트리 델타를 구하고,
  //   각 파티클에 동일한 델타 + 독립적 가우시안 노이즈를 더함
  //   (기존 방식: 파티클 전체를 robot0 주변으로 리셋 → 개별 위치 소실)

  ik_Xmoved = msg->x;
  ik_Ymoved = msg->y;

  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    // 이동 전 위치 저장
    ik_prev_x = robot0.x;
    ik_prev_y = robot0.y;

    robot0.move(ik_Xmoved, ik_Ymoved);

    // yaw 회전 + odom 스케일이 적용된 픽셀 델타
    ik_dx = robot0.x - ik_prev_x;
    ik_dy = robot0.y - ik_prev_y;
  }

  // 노이즈: 이동량에 비례 + 최소값 (튜닝 가능)
  ik_motion     = sqrt(ik_dx * ik_dx + ik_dy * ik_dy);
  ik_noise_sigma = ik_motion * 0.2 + 1.0;

  for (int i = 0; i < particle_num; i++)
  {
    pt[i].x += ik_dx + Likelihood.gaussianRandom(0, ik_noise_sigma);
    pt[i].y += ik_dy + Likelihood.gaussianRandom(0, ik_noise_sigma);
    // 필드 경계 클램핑
    if (pt[i].x < IK_PX_MIN_X) pt[i].x = IK_PX_MIN_X;
    if (pt[i].x > IK_PX_MAX_X) pt[i].x = IK_PX_MAX_X;
    if (pt[i].y < IK_PX_MIN_Y) pt[i].y = IK_PX_MIN_Y;
    if (pt[i].y > IK_PX_MAX_Y) pt[i].y = IK_PX_MAX_Y;
  }
}

void QNode::gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg)
{
  // PRE CONDITION : msg(gamecontroller)
  // POST CONDITION : mySide, penalty, position, robotNum
  // purpose : gamecontroller에서 콜백 받아온 경기 데이터를 적용

  gameMSG.mySide = msg->myside;
  gameMSG.penalty = msg->penalty;
  gameMSG.position = msg->position;
  gameMSG.robotNum = msg->robotnum;
  if (gameMSG.penalty != 0)
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    if (gameMSG.mySide)
    {
      if (gameMSG.position == 1)
      {
        robot0.x = FieldCoord::toPixelX(3.5);
        robot0.y = FieldCoord::toPixelY(-FieldCoord::B / 2);
      }
      else
      {
        robot0.x = FieldCoord::toPixelX(2.5);
        robot0.y = FieldCoord::toPixelY(-FieldCoord::B / 2);
      }
      for (int i = 0; i < particle_num; i++)
      {
        Likelihood.random_point(pt[i], robot0.x, robot0.y, particle_range);
      }
    }
    else
    {
      if (gameMSG.position == 1)
      {
        robot0.x = FieldCoord::toPixelX(-3.5);
        robot0.y = FieldCoord::toPixelY(-FieldCoord::B / 2);
      }
      else
      {
        robot0.x = FieldCoord::toPixelX(-2.5);
        robot0.y = FieldCoord::toPixelY(-FieldCoord::B / 2);
      }
      for (int i = 0; i < particle_num; i++)
      {
        Likelihood.random_point(pt[i], robot0.x, robot0.y, particle_range);
      }
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0;
    master_target_y = 0;
  }
}

void QNode::udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg)
{
  // PRE CONDITION : msg(udpcom)
  // POST CONDITION : robotX.n, ballX.n,
  // purpose : udp통신으로 받아온 다른 로봇 데이터를 콜백하여 내부 변수에 적용
  if (msg->robotnum == 1)
  {
    robot1.x = msg->localx;
    robot1.y = msg->localy;
    robot1.z = msg->localyaw;
    robot1.b = 1;
    robot1.state = msg->robotcase;

    ball1.x = msg->ballx;
    ball1.y = msg->bally;
    ball1.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 2)
  {
    robot2.x = msg->localx;
    robot2.y = msg->localy;
    robot2.z = msg->localyaw;
    robot2.b = 1;
    robot2.state = msg->robotcase;

    ball2.x = msg->ballx;
    ball2.y = msg->bally;
    ball2.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 3)
  {
    robot3.x = msg->localx;
    robot3.y = msg->localy;
    robot3.z = msg->localyaw;
    robot3.b = 1;
    robot3.state = msg->robotcase;

    ball3.x = msg->ballx;
    ball3.y = msg->bally;
    ball3.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
  if (msg->robotnum == 4)
  {
    robot4.x = msg->localx;
    robot4.y = msg->localy;
    robot4.z = msg->localyaw;
    robot4.b = 1;
    robot4.state = msg->robotcase;

    ball4.x = msg->ballx;
    ball4.y = msg->bally;
    ball4.d = sqrt(pow(msg->localx - msg->ballx, 2) + pow(msg->localy - msg->bally, 2));
  }
}

void QNode::masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg)
{
  // PRE CONDITION : msg(master)
  // POST CONDITION : master_target_N
  // purpose : 마스터에서 받은 타겟 좌표 데이터 적용

  master_target_x = msg->targetx;
  master_target_y = msg->targety;
}
