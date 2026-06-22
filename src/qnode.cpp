/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date May 2026
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robocup_localization/qnode.hpp"
#include "../include/field_coordinate/field_coordinate.hpp"

/*****************************************************************************
** QNode
*****************************************************************************/

QNode::QNode(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robocup_localization");
  this->start();

  getParams();

  imu_sub_ = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().best_effort(), std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  vision_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvision>(vision_topic, 1, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvisionfeature>(vision_feature_topic, 4, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(ik_topic, 1, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<gamecontroller::msg::Gamecontroldata>(game_control_topic, 5, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>(udp_topic, 5, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization>(master_topic, 5, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<robocup_localization::msg::RobocupLocalization>(local_pub_topic, 5);
  pan_angle_comp_pub_ = node->create_publisher<vision_interfaces::msg::PanAngleCompensation>("/pan_angle_compensation", 5);

  if (local_helper_ || corner_kick_receiver_)
    technical_challenge_pub_ = node->create_publisher<robocup_localization::msg::TechnicalChallenge>("/technical_challenge", 5);

  if (local_kicker_)
    blind_robot_sub_ = node->create_subscription<robocup_localization::msg::TechnicalChallenge>(
        "/technical_challenge", 5, std::bind(&QNode::blindRobotCallback, this, std::placeholders::_1));

  if (robot_feature == "adult")
  {
    Likelihood.filter.OBS_FILTER_DIST_M = 999999.0; // 관측 pre-filter 거리 임계값 false
  }

  loop_hz = corner_kick_receiver_ ? 50 : 20;

  if (soccer_mode_)
    startMclProcessing();
}

QNode::~QNode()
{
  {
    std::lock_guard<std::mutex> lock(mcl_mutex_);
    mcl_running_ = false;
  }
  mcl_cv_.notify_one();
  if (mcl_thread_.joinable())
    mcl_thread_.join();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::startMclProcessing()
{
  mcl_running_ = true;
  mcl_thread_ = std::thread(&QNode::mclProcessing, this);
}

void QNode::mclProcessing()
{
  while (mcl_running_)
  {
    std::unique_lock<std::mutex> lock(mcl_mutex_);
    while (!mcl_pending_ && mcl_running_)
      mcl_cv_.wait(lock);
    if (!mcl_running_)
      break;
    mcl_pending_ = false;
    lock.unlock();

    {
      std::lock_guard<std::mutex> pt_lock(pt_mutex);
      Likelihood.featureCalc(this, particle_range);
    }
  }
}

void QNode::run()
{

  rclcpp::WallRate loop_rate(loop_hz);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    yaw_compensation();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::visionFeatureCallback(const vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg)
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
    Likelihood.vision_point.CONFIDENCE = msg->corner_confidence[i];
    Likelihood.vision_point.DISTANCE = msg->corner_distance[i];
    Likelihood.vision_point.RAW_X_MM = msg->corner_point_vec_x[i];
    Likelihood.vision_point.RAW_Y_MM = msg->corner_point_vec_y[i];
    Likelihood.vision_point.POINT_VEC_X = (-1) * (msg->corner_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z)*M_PI / 180) + msg->corner_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z)*M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y = (msg->corner_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z)*M_PI / 180) - msg->corner_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z)*M_PI / 180));
    Likelihood.vision_point.STD_X = robot0_x;
    Likelihood.vision_point.STD_Y = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 0; // L
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
    Likelihood.vision_point.CONFIDENCE = msg->t_confidence[i];
    Likelihood.vision_point.DISTANCE = msg->t_distance[i];
    Likelihood.vision_point.RAW_X_MM = msg->t_point_vec_x[i];
    Likelihood.vision_point.RAW_Y_MM = msg->t_point_vec_y[i];
    Likelihood.vision_point.POINT_VEC_X = (-1) * (msg->t_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z)*M_PI / 180) + msg->t_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z)*M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y = (msg->t_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z)*M_PI / 180) - msg->t_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z)*M_PI / 180));
    Likelihood.vision_point.STD_X = robot0_x;
    Likelihood.vision_point.STD_Y = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 1; // T
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
    Likelihood.vision_point.CONFIDENCE = msg->cross_confidence[i];
    Likelihood.vision_point.DISTANCE = msg->cross_distance[i];
    Likelihood.vision_point.RAW_X_MM = msg->cross_point_vec_x[i];
    Likelihood.vision_point.RAW_Y_MM = msg->cross_point_vec_y[i];
    Likelihood.vision_point.POINT_VEC_X = (-1) * (msg->cross_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_X * cos((robot0_z)*M_PI / 180) + msg->cross_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_X * sin((robot0_z)*M_PI / 180));
    Likelihood.vision_point.POINT_VEC_Y = (msg->cross_point_vec_x[i] / 1000.0 * FieldCoord::PX_PER_M_Y * sin((robot0_z)*M_PI / 180) - msg->cross_point_vec_y[i] / 1000.0 * FieldCoord::PX_PER_M_Y * cos((robot0_z)*M_PI / 180));
    Likelihood.vision_point.STD_X = robot0_x;
    Likelihood.vision_point.STD_Y = robot0_y;
    Likelihood.vision_point.FEATURE_TYPE = 2; // X

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
                           !msg->t_confidence.empty() ||
                           !msg->cross_confidence.empty();

  bool mcl_active = (gameMSG.state == 3) || (gameMSG.state == 1 && gameMSG.ready_count >= 2);
  if (soccer_mode_ && !local_helper_ && has_real_features && !Likelihood.vision_point_vect.empty() && visionMSG.flag_pan == 0 && mcl_active)
  {
    // RCLCPP_INFO(node->get_logger(),
    //   "[OBS] vect=%zu (L=%d T=%d X=%d) total_cnt=%d",
    //   Likelihood.vision_point_vect.size(),
    //   (int)msg->corner_confidence.size(),
    //   (int)msg->t_confidence.size(),
    //   (int)msg->cross_confidence.size(),
    //   vision_data_cnt);
    Likelihood.filter.setCircle(robot0_z, Likelihood.vision_point_vect);
    valid_observation_ = true;
    {
      std::lock_guard<std::mutex> lock(mcl_mutex_);
      mcl_pending_ = true;
    }
    mcl_cv_.notify_one();
  }
  else
  {
    valid_observation_ = false;
    Likelihood.vision_point_vect.clear();
    vision_data_cnt  = 0;
    vision_data_size = 0;
  }
}

void QNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(robot0_mutex);
  if (imu_ignore_count_ > 0)
  {
    imu_ignore_count_--;
    return;
  }

  if (local_helper_)
  {
    switch (local_helper_dir)
    {
    case 0:
      robot0.z = -90.0;
      break;
    case 1:
      robot0.z = 90.0;
      break;
    }
    return;
  }

  double raw_yaw = msg->yaw;
  ik_d_yaw = raw_yaw - robot0.z;
  if (ik_d_yaw > 180.0)
    ik_d_yaw -= 360.0;
  if (ik_d_yaw < -180.0)
    ik_d_yaw += 360.0;
  robot0.z = raw_yaw;
}

void QNode::visionCallback(const vision_interfaces::msg::Robocupvision::SharedPtr msg)
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

  if (!msg->goal_post_pan_deg.empty())
  {
    goal_post_pan_deg_buf_ = msg->goal_post_pan_deg;
  }

  if (local_helper_ && !visionMSG.ROBOT_VEC_X.empty())
  {
    float robot0_x, robot0_y, robot0_z;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0_x = robot0.x;
      robot0_y = robot0.y;
      robot0_z = robot0.z;
    }
    RH::computeBlindRobotCoord(
        visionMSG.ROBOT_VEC_X[0], visionMSG.ROBOT_VEC_Y[0],
        robot0_x, robot0_y, robot0_z,
        technical_challengeMSG.blind_robot_x,
        technical_challengeMSG.blind_robot_y);
  }
}

void QNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  if (local_kicker_)
    return;

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

    if (masterMSG.master_move2ball_phase == "FOOT_ALIGN")
    {
      ik_dx *= foot_align_odom_scale_;
      ik_dy *= foot_align_odom_scale_;
    }
  }

  // rot1/trans/rot2 모션 모델
  // ik_rot1  : robot-frame에서 이동 방향 (전진=0, 좌측=+90도)
  // ik_trans : world-frame 이동 거리 (픽셀)
  // rot2는 yaw를 IMU에서 별도 처리하므로 0으로 가정
  ik_trans = sqrt(ik_dx * ik_dx + ik_dy * ik_dy);

  std::lock_guard<std::mutex> pt_lock(pt_mutex);
  if (ik_trans < 0.1)
  {
    for (int i = 0; i < particle_num; i++)
    {
      if (noise_enable_)
      {
        pt[i].x += Likelihood.gaussianRandom(0, 0.1);
        pt[i].y += Likelihood.gaussianRandom(0, 0.1);
      }
      pt[i].z += ik_d_yaw;
      if (pt[i].z > 180.0)
        pt[i].z -= 360.0;
      if (pt[i].z < -180.0)
        pt[i].z += 360.0;
    }
  }
  else
  {
    ik_rot1 = atan2(ik_Ymoved, ik_Xmoved);

    double world_dir = atan2(ik_dy, ik_dx);

    for (int i = 0; i < particle_num; i++)
    {
      if (noise_enable_)
      {
        ik_trans_dev = ALPHA3 * ik_trans + ALPHA4 * fabs(ik_rot1);
        ik_rot1_dev = ALPHA1 * fabs(ik_rot1) + ALPHA2 * ik_trans;

        double n_trans = ik_trans + Likelihood.gaussianRandom(0, ik_trans_dev);
        double n_dir = world_dir + Likelihood.gaussianRandom(0, ik_rot1_dev);

        pt[i].x += n_trans * cos(n_dir);
        pt[i].y += n_trans * sin(n_dir);

        double yaw_noise = Likelihood.gaussianRandom(0, ALPHA5 * fabs(ik_d_yaw) + 0.1);
        pt[i].z += ik_d_yaw + yaw_noise;
      }
      else
      {
        pt[i].x += ik_trans * cos(world_dir);
        pt[i].y += ik_trans * sin(world_dir);
        pt[i].z += ik_d_yaw;
      }

      if (pt[i].z > 180.0)
        pt[i].z -= 360.0;
      if (pt[i].z < -180.0)
        pt[i].z += 360.0;

      if (pt[i].x < IK_PX_MIN_X)
        pt[i].x = IK_PX_MIN_X;
      if (pt[i].x > IK_PX_MAX_X)
        pt[i].x = IK_PX_MAX_X;
      if (pt[i].y < IK_PX_MIN_Y)
        pt[i].y = IK_PX_MIN_Y;
      if (pt[i].y > IK_PX_MAX_Y)
        pt[i].y = IK_PX_MAX_Y;
    }
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
  int prev_state = gameMSG.state;
  gameMSG.state = msg->state;
  if (prev_state != 1 && gameMSG.state == 1)
    gameMSG.ready_count++;
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
        Likelihood.random_point(pt[i], robot0.x, robot0.y, particle_range, robot0.z);
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
        Likelihood.random_point(pt[i], robot0.x, robot0.y, particle_range, robot0.z);
      }
    }
    robot0.TIME_STAMP.clear();
    masterMSG.master_target_x = 0;
    masterMSG.master_target_y = 0;
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

  masterMSG.master_target_x = msg->targetx;
  masterMSG.master_target_y = msg->targety;
  masterMSG.master_best_position_attack_x = msg->bestposition_x;
  masterMSG.master_best_position_attack_y = msg->bestposition_y;
  masterMSG.master_best_position_defend_x = msg->defendposition_x;
  masterMSG.master_best_position_defend_y = msg->defendposition_y;
  masterMSG.master_angle_to_target = msg->angle_to_target;
  masterMSG.master_move2ball_phase = msg->move2ball_phase;
}

void QNode::blindRobotCallback(const robocup_localization::msg::TechnicalChallenge::SharedPtr msg)
{
  if (msg->blind_robot_x == 0.0 && msg->blind_robot_y == 0.0)
    return;
  std::lock_guard<std::mutex> lock(robot0_mutex);
  robot0.x = static_cast<float>(msg->blind_robot_x);
  robot0.y = static_cast<float>(msg->blind_robot_y);
}

void QNode::yaw_compensation()
{
  if (goal_post_pan_deg_buf_.empty())
  {
    return;
  }

  if (yaw_correction_cooldown_ > 0)
  {
    yaw_correction_cooldown_--;
    return;
  }

  float robot0_x, robot0_y, robot0_z;
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot0_x = robot0.x;
    robot0_y = robot0.y;
    robot0_z = robot0.z;
  }

  double rx_m = FieldCoord::toFieldX(robot0_x);
  double ry_m = FieldCoord::toFieldY(robot0_y);

  double best_corrected_yaw = 0.0;
  double min_diff = 1e9;

  const FieldCoord::FeaturePoint goal_posts[2] = {
      FieldCoord::Left_Goalpost,
      FieldCoord::Right_Goalpost};

  for (int p = 0; p < (int)goal_post_pan_deg_buf_.size(); p++)
  {
    double pan_deg = goal_post_pan_deg_buf_[p];
    for (int g = 0; g < 2; g++)
    {
      double gx = goal_posts[g].x;
      double gy = goal_posts[g].y;
      double expected_yaw = atan2(gx - rx_m, gy - ry_m) * 180.0 / M_PI;
      double candidate_yaw = expected_yaw - pan_deg;
      while (candidate_yaw > 180.0)
        candidate_yaw -= 360.0;
      while (candidate_yaw < -180.0)
        candidate_yaw += 360.0;
      double diff = fabs(candidate_yaw - robot0_z);
      if (diff > 180.0)
        diff = 360.0 - diff;
      if (diff < min_diff)
      {
        min_diff = diff;
        best_corrected_yaw = candidate_yaw;
      }
    }
  }

  if (min_diff > YAW_DRIFT_THRESHOLD_DEG)
  {
    panCompMSG.target_yaw_deg = best_corrected_yaw;
    publish_pan_comp();
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0.z = best_corrected_yaw;
      imu_ignore_count_ = 20;
    }
    yaw_correction_cooldown_ = 100; // 5초 쿨다운
  }
  goal_post_pan_deg_buf_.clear();
}

void QNode::publish_localization(
    float ball_x, float ball_y,
    float ball_speed_x, float ball_speed_y,
    float robot_x, float robot_y,
    const std::vector<float> &obstacles_x,
    const std::vector<float> &obstacles_y)
{
  robocup_localization::msg::RobocupLocalization msg;
  msg.ball_x = ball_x;
  msg.ball_y = ball_y;
  msg.ball_speed_x = ball_speed_x;
  msg.ball_speed_y = ball_speed_y;
  msg.robot_x = robot_x;
  msg.robot_y = robot_y;
  msg.obstacles_x.assign(obstacles_x.begin(), obstacles_x.end());
  msg.obstacles_y.assign(obstacles_y.begin(), obstacles_y.end());
  localization_pub_->publish(msg);
}

void QNode::publish_pan_comp()
{
  vision_interfaces::msg::PanAngleCompensation msg;
  msg.target_yaw_deg = panCompMSG.target_yaw_deg;
  pan_angle_comp_pub_->publish(msg);
}

void QNode::publish_technical_challenge()
{
  if (!technical_challenge_pub_)
    return;
  robocup_localization::msg::TechnicalChallenge msg;
  msg.ball_in_approach = technical_challengeMSG.ball_in_approach;
  msg.blind_robot_x = technical_challengeMSG.blind_robot_x;
  msg.blind_robot_y = technical_challengeMSG.blind_robot_y;
  technical_challenge_pub_->publish(msg);
}
