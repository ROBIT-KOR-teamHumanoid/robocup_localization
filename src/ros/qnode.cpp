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

#include "robocup_localization/ros/qnode.hpp"
#include "field_coordinate/field_coordinate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

/*****************************************************************************
** QNode
*****************************************************************************/

QNode::QNode(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robocup_localization");

  getParams();

  imu_sub_ = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().best_effort(), std::bind(&QNode::imuCallback, this, std::placeholders::_1));
  vision_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvision>(vision_topic, 1, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvisionfeature>(vision_feature_topic, 4, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(ik_topic, 1, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<gamecontroller::msg::Gamecontroldata>(game_control_topic, 5, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>(udp_topic, 5, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization>(master_topic, 5, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<robocup_localization::msg::RobocupLocalization>(local_pub_topic, 5);

  if (robot_feature == "adult")
  {
    localization_engine_.setObservationFilterDistance(999999.0); // 관측 pre-filter 거리 임계값 false
  }


  if (soccer_mode_)
    startMclProcessing();

  this->start();
}

QNode::~QNode()
{
  // 1. run() 반복문 종료 요청
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  // 2. QThread가 실제로 종료될 때까지 대기
  if (isRunning())
  {
    wait();
  }

  // 3. MCL 작업 스레드 종료
  {
    std::lock_guard<std::mutex> lock(mcl_mutex_);
    mcl_running_ = false;
  }

  mcl_cv_.notify_one();

  if (mcl_thread_.joinable())
  {
    mcl_thread_.join();
  }
}

robocup_localization::app::LocalizationConfig
QNode::localizationConfig() const noexcept
{
  return localization_config_;
}

robocup_localization::app::LocalizationSnapshot
QNode::localizationSnapshot() const
{
  robocup_localization::app::LocalizationSnapshot snapshot;
  snapshot.world = localization_engine_.snapshot();
  snapshot.particle_filter = localization_engine_.particleFilterSnapshot();

  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    snapshot.robot.pose.x = robot0.x;
    snapshot.robot.pose.y = robot0.y;
    snapshot.robot.pose.yaw_deg = robot0.z;
    snapshot.robot.odom_forward = robot0.odom_fx;
    snapshot.robot.odom_backward = robot0.odom_bx;
    snapshot.robot.odom_left = robot0.odom_ly;
    snapshot.robot.odom_right = robot0.odom_ry;
    snapshot.robot.ik_x = robot0.now_ik_param_x;
    snapshot.robot.ik_y = robot0.now_ik_param_y;
    snapshot.robot.trail.reserve(robot0.TIME_STAMP.size());
    for (const auto& point : robot0.TIME_STAMP)
    {
      snapshot.robot.trail.push_back({point.X, point.Y});
    }
  }
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    snapshot.master = master_state_;
  }
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    snapshot.game = game_state_;
  }
  return snapshot;
}

void QNode::previewRobotPosition(float x, float y)
{
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot0.x = x;
    robot0.y = y;
    robot0.TIME_STAMP.clear();
  }
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    master_state_.master_target_x = 0;
    master_state_.master_target_y = 0;
  }
}

void QNode::placeRobot(float x, float y, float particle_range)
{
  float yaw_deg;
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot0.x = x;
    robot0.y = y;
    robot0.TIME_STAMP.clear();
    yaw_deg = robot0.z;
  }
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    master_state_.master_target_x = 0;
    master_state_.master_target_y = 0;
  }
  resetParticles(x, y, yaw_deg, particle_range);
}

void QNode::setRobotOdometryCalibration(
    float forward, float backward, float left, float right)
{
  std::lock_guard<std::mutex> lock(robot0_mutex);
  robot0.odom_fx = forward;
  robot0.odom_bx = backward;
  robot0.odom_ly = left;
  robot0.odom_ry = right;
}

void QNode::setManualBall(float x, float y)
{
  localization_engine_.setManualBall(x, y);
}

void QNode::clearManualBall()
{
  localization_engine_.clearManualBall();
}

void QNode::clearTeammates()
{
  localization_engine_.clearTeammates();
}

void QNode::addManualObstacle(float x, float y)
{
  localization_engine_.addManualObstacle(x, y);
}

void QNode::removeLastManualObstacle()
{
  localization_engine_.removeLastManualObstacle();
}

void QNode::setParticleRange(float range)
{
  particle_range_ = range;
  localization_engine_.setParticleRange(range);
}

void QNode::resetParticles(
    float x, float y, float yaw_deg, float range)
{
  localization_engine_.resetParticles(x, y, yaw_deg, range);
}

void QNode::clearParticleFilterVision()
{
  localization_engine_.clearParticleFilterVision();
}

void QNode::startMclProcessing()
{
  {
    std::lock_guard<std::mutex> lock(mcl_mutex_);
    mcl_running_ = true;
    mcl_pending_ = false;
  }

  mcl_thread_ = std::thread(&QNode::mclProcessing, this);
}

void QNode::mclProcessing()
{
  while (true)
  {
    {
      std::unique_lock<std::mutex> lock(mcl_mutex_);
      mcl_cv_.wait(lock, [this]
      {
        return mcl_pending_ || !mcl_running_;
      });

      if (!mcl_running_)
      {
        break;
      }

      mcl_pending_ = false;
    }

    float robot_x;
    float robot_y;
    float robot_yaw_deg;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot_x = robot0.x;
      robot_y = robot0.y;
      robot_yaw_deg = robot0.z;
    }

    const FeatureCalcResult result =
        localization_engine_.correctParticles(robot_x, robot_y, robot_yaw_deg);

    if (result.pose_updated)
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0.x = result.robot_x;
      robot0.y = result.robot_y;
    }
  }
}
void QNode::run()
{
  rclcpp::WallRate loop_rate(localization_config_.ros_spin_hz);
  auto last_update = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - last_update;
    last_update = now;
    localization_engine_.update(elapsed.count());
    publishWorldModelSnapshot(localization_engine_.snapshot());

    loop_rate.sleep();
  }

  Q_EMIT rosShutDown();
}

void QNode::publishWorldModelSnapshot(
    const robocup_localization::core::WorldModelSnapshot& snapshot)
{
  std::vector<float> obstacle_x;
  std::vector<float> obstacle_y;

  for (const auto& obstacle : snapshot.obstacles)
  {
    if (obstacle.nocnt == 0)
    {
      obstacle_x.push_back(obstacle.x);
      obstacle_y.push_back(obstacle.y);
    }
    else
    {
      obstacle_x.push_back(0.0f);
      obstacle_y.push_back(0.0f);
    }
  }

  for (const auto& obstacle : snapshot.manual_obstacles)
  {
    obstacle_x.push_back(obstacle.x);
    obstacle_y.push_back(obstacle.y);
  }

  float robot_x;
  float robot_y;
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot_x = robot0.x;
    robot_y = robot0.y;
  }

  const BALL& selected_ball = snapshot.ball;
  publish_localization(
      selected_ball.x,
      selected_ball.y,
      selected_ball.x +
          selected_ball.speed_x * snapshot.ball_prediction_level,
      selected_ball.y +
          selected_ball.speed_y * snapshot.ball_prediction_level,
      robot_x,
      robot_y,
      obstacle_x,
      obstacle_y);
}

void QNode::visionFeatureCallback(const vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg)
{
  float robot0_x;
  float robot0_y;
  float robot0_z;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    robot0_x = robot0.x;
    robot0_y = robot0.y;
    robot0_z = robot0.z;
  }

  std::vector<VISION_POINT> observations;
  observations.reserve(
      msg->corner_confidence.size() +
      msg->t_confidence.size() +
      msg->cross_confidence.size());

  const double yaw_rad = robot0_z * M_PI / 180.0;
  const auto append_features =
      [&](const auto& confidences,
          const auto& distances,
          const auto& raw_x_values,
          const auto& raw_y_values,
          const auto& ball_angles,
          const auto& ball_distances,
          int feature_type)
      {
        const std::size_t count = std::min(
            {confidences.size(), distances.size(),
             raw_x_values.size(), raw_y_values.size()});
        for (std::size_t i = 0; i < count; ++i)
        {
          VISION_POINT observation{};
          observation.CONFIDENCE = confidences[i];
          observation.DISTANCE = distances[i];
          observation.RAW_X_MM = raw_x_values[i];
          observation.RAW_Y_MM = raw_y_values[i];
          observation.POINT_VEC_X = static_cast<int>(
              -(raw_x_values[i] / 1000.0 * FieldCoord::PX_PER_M_X *
                    std::cos(yaw_rad) +
                raw_y_values[i] / 1000.0 * FieldCoord::PX_PER_M_X *
                    std::sin(yaw_rad)));
          observation.POINT_VEC_Y = static_cast<int>(
              raw_x_values[i] / 1000.0 * FieldCoord::PX_PER_M_Y *
                  std::sin(yaw_rad) -
              raw_y_values[i] / 1000.0 * FieldCoord::PX_PER_M_Y *
                  std::cos(yaw_rad));
          observation.STD_X = static_cast<int>(robot0_x);
          observation.STD_Y = static_cast<int>(robot0_y);
          observation.FEATURE_TYPE = feature_type;
          observation.BALL_RELATIVE_ANGLE =
              i < ball_angles.size() ? ball_angles[i] : 0.0;
          observation.BALL_FEATURE_DISTANCE =
              i < ball_distances.size() ? ball_distances[i] : 0.0;
          observations.push_back(observation);
        }
      };

  append_features(
      msg->corner_confidence, msg->corner_distance,
      msg->corner_point_vec_x, msg->corner_point_vec_y,
      msg->corner_ball_relative_angle,
      msg->corner_ball_feature_distance, 0);
  append_features(
      msg->t_confidence, msg->t_distance,
      msg->t_point_vec_x, msg->t_point_vec_y,
      msg->t_ball_relative_angle,
      msg->t_ball_feature_distance, 1);
  append_features(
      msg->cross_confidence, msg->cross_distance,
      msg->cross_point_vec_x, msg->cross_point_vec_y,
      msg->cross_ball_relative_angle,
      msg->cross_ball_feature_distance, 2);

  bool mcl_active;
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    mcl_active = game_state_.state == 3 ||
        (game_state_.state == 1 && game_state_.ready_count >= 2);
  }
  const bool valid_observation =
      soccer_mode_ && !observations.empty() &&
      vision_pan_flag_ == 0 && mcl_active;

  localization_engine_.submitFeatures(
      std::move(observations), valid_observation, robot0_z);

  if (valid_observation)
  {
    {
      std::lock_guard<std::mutex> lock(mcl_mutex_);
      mcl_pending_ = true;
    }
    mcl_cv_.notify_one();
  }
}

void QNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(robot0_mutex);
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

  localization_engine_.setBallObservation(
      msg->ball_2d_x, msg->ball_2d_y);
  vision_pan_flag_ = msg->flag_pan;

  if (msg->ball_cam_x != 0 || msg->ball_cam_y != 0)
  {
    clearManualBall();
  }

  int robot_y_sign = 0;
  if (!msg->goal_post_pan_deg.empty())
  {
    goal_post_pan_deg_buf_ = msg->goal_post_pan_deg;
    float robot0_z;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      robot0_z = robot0.z;
    }
    robot_y_sign =
        Filter::computeYSign(robot0_z, goal_post_pan_deg_buf_);

    bool mirror_particles = false;
    {
      std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
      const double robot_y_m = FieldCoord::toFieldY(robot0.y);
      if (robot_y_sign != 0 && robot_y_sign * robot_y_m < 0.0)
      {
        robot0.y = static_cast<float>(FieldCoord::MAP_H) - robot0.y;
        mirror_particles = true;
      }
    }

    if (mirror_particles)
    {
      localization_engine_.mirrorParticlesY(static_cast<float>(FieldCoord::MAP_H));
    }
  }
  localization_engine_.setRobotYSign(robot_y_sign);

  robocup_localization::core::Pose2D robot_pose;
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot_pose.x = robot0.x;
    robot_pose.y = robot0.y;
    robot_pose.yaw_deg = robot0.z;
  }

  robocup_localization::core::VisionObservation observation;
  observation.ball_detected =
      msg->ball_2d_x != 0.0f || msg->ball_2d_y != 0.0f;
  observation.ball_x_mm = msg->ball_2d_x;
  observation.ball_y_mm = msg->ball_2d_y;
  observation.ball_distance_mm = msg->ball_d;
  observation.ball_speed_x = msg->ball_speed_x;
  observation.ball_speed_y = msg->ball_speed_y;
  observation.ball_prediction_level = msg->ball_speed_level;
  observation.obstacle_x_mm.assign(
      msg->robot_vec_x.begin(), msg->robot_vec_x.end());
  observation.obstacle_y_mm.assign(
      msg->robot_vec_y.begin(), msg->robot_vec_y.end());

  localization_engine_.updateVision(robot_pose, observation);
}

void QNode::ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg)
{
  ik_Xmoved = msg->x;
  ik_Ymoved = msg->y;

  bool foot_align;
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    foot_align = master_state_.master_move2ball_phase == "FOOT_ALIGN";
  }

  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    // 이동 전 위치 저장
    ik_prev_x = robot0.x;
    ik_prev_y = robot0.y;

    robot0.move(ik_Xmoved, ik_Ymoved);

    // yaw 회전 + odom 스케일이 적용된 픽셀 델타
    ik_dx = robot0.x - ik_prev_x;
    ik_dy = robot0.y - ik_prev_y;

    if (foot_align)
    {
      ik_dx *= foot_align_odom_scale_;
      ik_dy *= foot_align_odom_scale_;
    }
  }

  // 로봇 오도메트리에서 계산한 이동량만 ParticleFilter에 전달한다.
  ik_trans = std::sqrt(ik_dx * ik_dx + ik_dy * ik_dy);

  robocup_localization::core::ParticleMotion motion;
  motion.translation_px = ik_trans;
  motion.yaw_delta_deg = ik_d_yaw;
  motion.yaw_stddev = ALPHA5 * std::fabs(ik_d_yaw) + 0.1;

  if (ik_trans >= 0.1)
  {
    ik_rot1 = std::atan2(ik_Ymoved, ik_Xmoved);
    motion.world_direction_rad = std::atan2(ik_dy, ik_dx);

    ik_trans_dev = ALPHA3 * ik_trans + ALPHA4 * std::fabs(ik_rot1);
    ik_rot1_dev = ALPHA1 * std::fabs(ik_rot1) + ALPHA2 * ik_trans;
    motion.translation_stddev = ik_trans_dev;
    motion.direction_stddev = ik_rot1_dev;
  }

  localization_engine_.applyParticleMotion(motion);

}

void QNode::gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg)
{
  robocup_localization::app::GameStateSnapshot game;
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    game_state_.mySide = msg->myside;
    game_state_.penalty = msg->penalty;
    game_state_.position = msg->position;
    game_state_.robotNum = msg->robotnum;
    const int previous_state = game_state_.state;
    game_state_.state = msg->state;
    if (previous_state != 1 && game_state_.state == 1)
    {
      ++game_state_.ready_count;
    }
    game = game_state_;
  }

  if (game.penalty == 0)
  {
    return;
  }

  const double field_x = game.mySide
      ? (game.position == 1 ? 3.5 : 2.5)
      : (game.position == 1 ? -3.5 : -2.5);

  float robot_x;
  float robot_y;
  float robot_yaw_deg;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    robot0.x = FieldCoord::toPixelX(field_x);
    robot0.y = FieldCoord::toPixelY(-FieldCoord::B / 2);
    robot0.TIME_STAMP.clear();
    robot_x = robot0.x;
    robot_y = robot0.y;
    robot_yaw_deg = robot0.z;
  }

  resetParticles(robot_x, robot_y, robot_yaw_deg, particle_range_);
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    master_state_.master_target_x = 0;
    master_state_.master_target_y = 0;
  }
}

void QNode::udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg)
{
  if (msg->robotnum < 1 || msg->robotnum > 4)
  {
    return;
  }

  robocup_localization::core::TeammateObservation observation;
  observation.pose.x = msg->localx;
  observation.pose.y = msg->localy;
  observation.pose.yaw_deg = msg->localyaw;
  observation.state = msg->robotcase;
  observation.ball_x = msg->ballx;
  observation.ball_y = msg->bally;

  localization_engine_.updateTeammate(
      static_cast<std::size_t>(msg->robotnum - 1), observation);
}

void QNode::masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg)
{
  // PRE CONDITION : msg(master)
  // POST CONDITION : master_target_N
  // purpose : 마스터에서 받은 타겟 좌표 데이터 적용

  std::lock_guard<std::mutex> lock(master_state_mutex_);
  master_state_.master_target_x = msg->targetx;
  master_state_.master_target_y = msg->targety;
  master_state_.master_best_position_attack_x = msg->bestposition_x;
  master_state_.master_best_position_attack_y = msg->bestposition_y;
  master_state_.master_best_position_defend_x = msg->defendposition_x;
  master_state_.master_best_position_defend_y = msg->defendposition_y;
  master_state_.master_angle_to_target = msg->angle_to_target;
  master_state_.master_move2ball_phase = msg->move2ball_phase;
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

