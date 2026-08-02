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

#include "gui/qnode.hpp"
#include "field_profile/field_coordinate.hpp"
#include "field_profile/start_positions.hpp"
#include "utils/team_udp_receiver.hpp"
#include "pre_processing/vision_processing.hpp"
#include "utils/yaw_compensator.hpp"
#include "monte_carlo_localization/particle_reset.hpp"
#include "mlp_localization/mlp_dimension_adapter.hpp"

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
  vision_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvision>(vision_topic, 3, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvisionfeature>(vision_feature_topic, 4, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(ik_topic, 1, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<gamecontroller::msg::Gamecontroldata>(game_control_topic, 5, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>(udp_topic, 5, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization>(master_topic, 5, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<robocup_localization::msg::RobocupLocalization>(local_pub_topic, 5);
  pan_angle_comp_pub_ = node->create_publisher<vision_interfaces::msg::PanAngleCompensation>("/pan_angle_compensation", 5);
  

  loop_hz = 20;

  // ===== Learned (ONNX) localization 초기화 =====
  if (learned_mlp_enable_ || learned_reloc_enable_)
  {
    learned_loc::LearnedConfig lc;
    lc.mlp_enable = learned_mlp_enable_;
    lc.reloc_enable = learned_reloc_enable_;
    lc.mlp_model_path = learned_mlp_model_;
    lc.reloc_model_path = learned_reloc_model_;
    lc.reloc_topk = learned_reloc_topk_;
    lc.sign_right = learned_sign_right_;
    lc.sign_forward = learned_sign_forward_;

    if (!learned_.init(lc))
    {
      RCLCPP_WARN(node->get_logger(),
                  "[learned] ONNX 로드 실패: %s (해당 추론 비활성)",
                  learned_.last_error().c_str());
    }
    else
    {
      RCLCPP_INFO(node->get_logger(),
                  "[learned] mlp=%s reloc=%s",
                  learned_.mlp_ready() ? "on" : "off",
                  learned_.reloc_ready() ? "on" : "off");
    }
  }

  if (soccer_mode_)
    mcl_.start();
}

QNode::~QNode()
{
  // 정리 순서가 중요하다.
  //  1) ROS 입력을 먼저 끊는다 — shutdown() 이 run() 의 while(ok()) 를 끝낸다
  //  2) QThread 가 실제로 빠져나올 때까지 기다린다. 이걸 안 하면 QThread 가
  //     "Destroyed while thread is still running" 으로 terminate 하거나,
  //     이미 파괴된 멤버를 콜백이 만진다
  //  3) 그 다음 MCL 워커를 세운다 — 콜백이 끝난 뒤라야 notifyObservation 이
  //     더 들어오지 않는다
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  // run() 은 20Hz 루프라 최대 50ms 정도 걸린다.
  if (isRunning() && !wait(2000))
  {
    RCLCPP_WARN(node->get_logger(), "[QNode] ROS 스레드가 제때 종료되지 않음");
  }

  mcl_.stop();
}

void QNode::run()
{

  rclcpp::WallRate loop_rate(loop_hz);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    // yaw_compensation();
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
    robot0_z = robot0.yaw_deg;
  }

  const VisionProcessing::Pose pose{robot0_x, robot0_y, robot0_z};

  // 특징점 세 타입을 관측 목록에 덧붙인다. 변환 규칙은 동일하고 필드 이름만 다르다.
  vision_data_cnt += VisionProcessing::append(
      Likelihood.vision_point_vect,
      msg->corner_confidence, msg->corner_distance,
      msg->corner_point_vec_x, msg->corner_point_vec_y,
      msg->corner_ball_relative_angle, msg->corner_ball_feature_distance,
      FieldCoord::L_SHAPE, pose);

  vision_data_cnt += VisionProcessing::append(
      Likelihood.vision_point_vect,
      msg->t_confidence, msg->t_distance,
      msg->t_point_vec_x, msg->t_point_vec_y,
      msg->t_ball_relative_angle, msg->t_ball_feature_distance,
      FieldCoord::T_SHAPE, pose);

  vision_data_cnt += VisionProcessing::append(
      Likelihood.vision_point_vect,
      msg->cross_confidence, msg->cross_distance,
      msg->cross_point_vec_x, msg->cross_point_vec_y,
      msg->cross_ball_relative_angle, msg->cross_ball_feature_distance,
      FieldCoord::X_SHAPE, pose);

  vision_data_size += msg->corner_confidence.size() +
                      msg->t_confidence.size() +
                      msg->cross_confidence.size();

  bool has_real_features = !msg->corner_confidence.empty() ||
                           !msg->t_confidence.empty() ||
                           !msg->cross_confidence.empty();

  // gamecontroller 없이 동작: 게임 상태 게이팅 무력화 (되돌리려면 아래 원식 복원)
  // bool mcl_active = (gameMSG.state == 3) || (gameMSG.state == 1 && gameMSG.ready_count >= 2);
  bool mcl_active = true;
  if (soccer_mode_ && has_real_features && !Likelihood.vision_point_vect.empty() && visionMSG.flag_pan == 0 && mcl_active)
  {
    // RCLCPP_INFO(node->get_logger(),
    //   "[OBS] vect=%zu (L=%d T=%d X=%d) total_cnt=%d",
    //   Likelihood.vision_point_vect.size(),
    //   (int)msg->corner_confidence.size(),
    //   (int)msg->t_confidence.size(),
    //   (int)msg->cross_confidence.size(),
    //   vision_data_cnt);
    Likelihood.filter.setCircle(robot0_z, Likelihood.vision_point_vect);
    // 공 위치도 이 프레임 시점의 값으로 함께 넘긴다. 워커가 나중에 깨어나
    // visionMSG 를 읽으면 다음 프레임 값일 수 있다.
    mcl_.ball_mm_x = visionMSG.Ball_2d_X;
    mcl_.ball_mm_y = visionMSG.Ball_2d_Y;
    valid_observation_ = true;
    mcl_.notifyObservation();
  }
  else
  {
    valid_observation_ = false;
    Likelihood.vision_point_vect.clear();
    vision_data_cnt  = 0;
    vision_data_size = 0;
  }

  // ===== 학습형(ONNX) 추론: vision_feature 프레임과 동기 실행 =====
  if (learned_mlp_enable_ || learned_reloc_enable_)
  {
    if (has_real_features)
    {
      runLearnedInference(msg, robot0_z);
    }
    else
    {
      // 특징점이 하나도 없으면 obs가 전부 valid=0 + yaw만 남아 의미 없는
      // pose가 나온다. 추론을 건너뛰고 직전 결과를 무효화한다.
      std::lock_guard<std::mutex> lock(learned_mutex_);
      learned_mlp_result_ = learned_loc::MlpResult{};
      learned_reloc_result_ = learned_loc::RelocResult{};
    }
  }
}

void QNode::runLearnedInference(
    const vision_interfaces::msg::Robocupvisionfeature::SharedPtr &msg,
    double yaw_deg)
{
  const auto f = MlpDimensionAdapter::fromMessage(*msg);

  const auto obs =
      learned_.buildObs(f.corner, f.t_shape, f.cross, f.goalpost, yaw_deg);

  learned_loc::MlpResult mlp;
  learned_loc::RelocResult reloc;

  if (learned_mlp_enable_ && learned_.mlp_ready())
    mlp = learned_.runMlp(obs);
  if (learned_reloc_enable_ && learned_.reloc_ready())
    reloc = learned_.runReloc(obs);

  {
    std::lock_guard<std::mutex> lock(learned_mutex_);
    if (learned_mlp_enable_)
      learned_mlp_result_ = mlp;
    if (learned_reloc_enable_)
      learned_reloc_result_ = reloc;
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

  double raw_yaw = msg->yaw;
  ik_d_yaw = raw_yaw - robot0.yaw_deg;
  if (ik_d_yaw > 180.0)
    ik_d_yaw -= 360.0;
  if (ik_d_yaw < -180.0)
    ik_d_yaw += 360.0;
  robot0.yaw_deg = raw_yaw;
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

  Likelihood.robot_y_sign = 0;
  if (!msg->goal_post_pan_deg.empty())
  {
    goal_post_pan_deg_buf_ = msg->goal_post_pan_deg;
    float robot0_z;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0_z = robot0.yaw_deg;
    }
    Likelihood.robot_y_sign = YawCompensator::computeYSign(robot0_z, goal_post_pan_deg_buf_);

    // y부호 불일치 시 robot0.y + 파티클 전체 y=0 기준 플립
    // 락 순서 pt -> robot0 (예전엔 반대라 순환의 한 변이었다)
    {
      std::lock_guard<std::mutex> pt_lock(pt_mutex);
      std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
      double ry_m = FieldCoord::toFieldY(robot0.y);
      if (Likelihood.robot_y_sign != 0 && Likelihood.robot_y_sign * ry_m < 0.0)
      {
        robot0.y = static_cast<float>(FieldCoord::MIRROR_Y_PX) - robot0.y;
        ParticleReset::mirrorAboutMidline(pt, particle_num);
      }
    }
  }

}

void QNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  const bool foot_align = (masterMSG.master_move2ball_phase == "FOOT_ALIGN");

  MotionModel::Delta delta;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    delta = motion_.applyOdometry(robot0, msg->x, msg->y, foot_align);
  }

  std::lock_guard<std::mutex> pt_lock(pt_mutex);
  motion_.propagate(pt, particle_num, delta, ik_d_yaw);
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
    // 반칙 후 복귀: 진영·포지션으로 정해진 자리로 옮기고 파티클을 다시 뿌린다.
    const auto p = StartPositions::penaltyReturn(gameMSG.position, gameMSG.mySide);
    mcl_.placeRobot(p.x, p.y, particle_range);

    masterMSG.master_target_x = 0;
    masterMSG.master_target_y = 0;
  }
}

void QNode::udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg)
{
  // PRE CONDITION : msg(udpcom)
  // POST CONDITION : robotX.n, ballX.n,
  // purpose : udp통신으로 받아온 다른 로봇 데이터를 team_udp_receiver로 위임하여 반영
  switch (msg->robotnum)
  {
  case 1: team_udp_receiver::receive(msg, robot1, ball1); break;
  case 2: team_udp_receiver::receive(msg, robot2, ball2); break;
  case 3: team_udp_receiver::receive(msg, robot3, ball3); break;
  case 4: team_udp_receiver::receive(msg, robot4, ball4); break;
  default: break;
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
    robot0_z = robot0.yaw_deg;
  }

  const YawCompensator::Result r = YawCompensator::estimate(
      FieldCoord::toFieldX(robot0_x), FieldCoord::toFieldY(robot0_y),
      robot0_z, goal_post_pan_deg_buf_);

  // 드리프트가 임계 이상일 때만 보정. 작은 차이는 노이즈로 본다.
  if (r.valid && r.drift_deg > YAW_DRIFT_THRESHOLD_DEG)
  {
    panCompMSG.target_yaw_deg = r.corrected_yaw_deg;
    publish_pan_comp();
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0.yaw_deg = r.corrected_yaw_deg;
      imu_ignore_count_ = 20;  // 보정 직후 IMU 델타 무시
    }
    yaw_correction_cooldown_ = 100; // 5초 쿨다운
  }
  goal_post_pan_deg_buf_.clear();
}

void QNode::publish_localization()
{
  robocup_localization::msg::RobocupLocalization msg;

  msg.ball_x = ball.x;
  msg.ball_y = ball.y;
  // 이름은 speed 지만 실제로는 speed_level 만큼 앞선 **예측 위치**다.
  // (main_window 가 이 점까지 선을 그린다) — 기존 동작 유지.
  msg.ball_speed_x = ball.x + ball.speed_x * visionMSG.Ball_speed_level;
  msg.ball_speed_y = ball.y + ball.speed_y * visionMSG.Ball_speed_level;

  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    msg.robot_x = robot0.x;
    msg.robot_y = robot0.y;
  }

  // 감지 중인 장애물만 좌표를 싣고, 놓친 것은 (0,0) 으로 자리를 유지한다
  // — 수신 측이 인덱스로 장애물을 구분하기 때문이다.
  for (const auto &obs : obstacles)
  {
    const bool tracked = (obs.nocnt == 0);
    msg.obstacles_x.push_back(tracked ? obs.x : 0.0f);
    msg.obstacles_y.push_back(tracked ? obs.y : 0.0f);
  }
  for (const auto &obs : debug_obstacles)
  {
    msg.obstacles_x.push_back(obs.x);
    msg.obstacles_y.push_back(obs.y);
  }

  localization_pub_->publish(msg);
}

void QNode::publish_pan_comp()
{
  vision_interfaces::msg::PanAngleCompensation msg;
  msg.target_yaw_deg = panCompMSG.target_yaw_deg;
  pan_angle_comp_pub_->publish(msg);
}

