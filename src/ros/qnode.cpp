#include "robocup_localization/ros/qnode.hpp"
#include "field_coordinate/field_coordinate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace
{

// Kidnap verification must use this same conversion as the MCL.
std::vector<VISION_POINT> buildVisionPoints(
    const vision_interfaces::msg::Robocupvisionfeature& msg,
    float robot_x, float robot_y, float yaw_deg)
{
  std::vector<VISION_POINT> observations;
  observations.reserve(
      msg.corner_confidence.size() +
      msg.t_confidence.size() +
      msg.cross_confidence.size());

  const double yaw_rad = yaw_deg * M_PI / 180.0;
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
          observation.STD_X = static_cast<int>(robot_x);
          observation.STD_Y = static_cast<int>(robot_y);
          observation.FEATURE_TYPE = feature_type;
          observation.BALL_RELATIVE_ANGLE = 0.0;
          if (i < ball_angles.size())
          {
            observation.BALL_RELATIVE_ANGLE = ball_angles[i];
          }
          observation.BALL_FEATURE_DISTANCE = 0.0;
          if (i < ball_distances.size())
          {
            observation.BALL_FEATURE_DISTANCE = ball_distances[i];
          }
          observations.push_back(observation);
        }
      };

  append_features(
      msg.corner_confidence, msg.corner_distance,
      msg.corner_point_vec_x, msg.corner_point_vec_y,
      msg.corner_ball_relative_angle,
      msg.corner_ball_feature_distance, 0);
  append_features(
      msg.t_confidence, msg.t_distance,
      msg.t_point_vec_x, msg.t_point_vec_y,
      msg.t_ball_relative_angle,
      msg.t_ball_feature_distance, 1);
  append_features(
      msg.cross_confidence, msg.cross_distance,
      msg.cross_point_vec_x, msg.cross_point_vec_y,
      msg.cross_ball_relative_angle,
      msg.cross_ball_feature_distance, 2);

  return observations;
}

}


QNode::QNode(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("robocup_localization");

  getParams();

  // Depth > 1: spin runs at ros_spin_hz, and every sample in between feeds the yaw history.
  imu_sub_ = node->create_subscription<humanoid_interfaces::msg::ImuMsg>(
      imu_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](const humanoid_interfaces::msg::ImuMsg::SharedPtr msg,
             const rclcpp::MessageInfo& info)
      {
        imuCallback(msg, info);
      });
  vision_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvision>(vision_topic, 1, std::bind(&QNode::visionCallback, this, std::placeholders::_1));
  vision_feature_sub_ = node->create_subscription<vision_interfaces::msg::Robocupvisionfeature>(vision_feature_topic, 1, std::bind(&QNode::visionFeatureCallback, this, std::placeholders::_1));

  // Odometry messages are increments, so a dropped message is lost motion: keep a deep queue.(BWS)odometry 메시지 유실 tips를 참고할 것. 
  ik_sub_ = node->create_subscription<humanoid_interfaces::msg::IkCoordMsg>(ik_topic, 100, std::bind(&QNode::ikCallback, this, std::placeholders::_1));
  game_control_sub_ = node->create_subscription<gamecontroller::msg::Gamecontroldata>(game_control_topic, 5, std::bind(&QNode::gameControlCallback, this, std::placeholders::_1));
  udp_sub_ = node->create_subscription<humanoid_interfaces::msg::Udp2master>(udp_topic, 5, std::bind(&QNode::udpCallback, this, std::placeholders::_1));
  master_sub_ = node->create_subscription<humanoid_interfaces::msg::Master2localization>(master_topic, 5, std::bind(&QNode::masterCallback, this, std::placeholders::_1));

  localization_pub_ = node->create_publisher<robocup_localization::msg::RobocupLocalization>(local_pub_topic, 5);
  // Field frame in metres; published only for valid MLP estimates.
  mlp_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(local_pub_topic + "/mlp", 5);

  initInference();

  startMclProcessing();

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  if (isRunning())
  {
    wait();
  }

  {
    std::lock_guard<std::mutex> lock(mcl_mutex_);
    mcl_running_ = false;
  }

  mcl_cv_.notify_one();

  if (mcl_thread_.joinable())
  {
    mcl_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(inference_job_mutex_);
    inference_running_ = false;
  }

  inference_cv_.notify_one();

  if (inference_thread_.joinable())
  {
    inference_thread_.join();
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
    snapshot.robot.pose = robot0;
    snapshot.robot.odom_forward = robot0.odom_fx;
    snapshot.robot.odom_backward = robot0.odom_bx;
    snapshot.robot.odom_left = robot0.odom_ly;
    snapshot.robot.odom_right = robot0.odom_ry;
    snapshot.robot.odom_metric =
        robot0.odom_input == ROBOT::OdometryInput::Metric;
    snapshot.robot.ik_x = robot0.now_ik_param_x;
    snapshot.robot.ik_y = robot0.now_ik_param_y;
    snapshot.robot.trail = robot0.trail;
  }
  {
    std::lock_guard<std::mutex> lock(master_state_mutex_);
    snapshot.master = master_state_;
  }
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    snapshot.game = game_state_;
  }
  {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    snapshot.inference = inference_state_;
  }
  return snapshot;
}

void QNode::initInference()
{
  localization_config_.inference = {};

  if (!inference_config_.mlp_enable && !inference_config_.reloc_enable)
  {
    return;
  }

  if (inference_config_.mlp_enable)
  {
    // An unknown gate means an unknown training condition, so the MLP is disabled.
    if (robocup_localization::inference::readFeatureMinGate(
            inference_config_.mlp_model_path, inference_config_.feature_min))
    {
      RCLCPP_INFO(
          node->get_logger(), "[inference] feature_min=%d (%s.json)",
          inference_config_.feature_min,
          inference_config_.mlp_model_path.c_str());
    }
    else
    {
      RCLCPP_ERROR(
          node->get_logger(),
          "[inference] %s.json 에서 feature_min_gate 를 읽지 못해 MLP 를 끕니다",
          inference_config_.mlp_model_path.c_str());
      inference_config_.mlp_enable = false;
    }
  }

  if (!inference_.init(inference_config_))
  {
    RCLCPP_ERROR(
        node->get_logger(), "[inference] 초기화 실패: %s",
        inference_.lastError().c_str());
  }

  localization_config_.inference.mlp =
      inference_config_.mlp_enable && inference_.mlpReady();
  localization_config_.inference.reloc =
      inference_config_.reloc_enable && inference_.relocReady();

  RCLCPP_INFO(
      node->get_logger(), "[inference] mlp=%s reloc=%s",
      localization_config_.inference.mlp ? "on" : "off",
      localization_config_.inference.reloc ? "on" : "off");

  if (localization_config_.inference.mlp || localization_config_.inference.reloc)
  {
    startInference();
  }
}

void QNode::startInference()
{
  {
    std::lock_guard<std::mutex> lock(inference_job_mutex_);
    inference_running_ = true;
    inference_pending_ = false;
  }

  inference_thread_ = std::thread(&QNode::inferenceProcessing, this);
}

void QNode::inferenceProcessing()
{
  while (true)
  {
    InferenceJob job;
    {
      std::unique_lock<std::mutex> lock(inference_job_mutex_);
      inference_cv_.wait(lock, [this]
      {
        return inference_pending_ || !inference_running_;
      });

      if (!inference_running_)
      {
        break;
      }

      job = std::move(inference_job_);
      inference_job_ = {};
      inference_pending_ = false;
    }

    runInference(job);
  }
}

void QNode::runInference(const InferenceJob& job)
{
  robocup_localization::inference::MlpResult mlp;
  robocup_localization::inference::RelocResult reloc;

  const auto started = std::chrono::steady_clock::now();
  bool ran = false;

  int n_msg = 0;
  int n_valid = 0;
  int n_arc_dropped = 0;

  // Decided in the worker, not the callback: clearing early would let a still-running
  // older frame overwrite the result later.
  if (job.has_features && job.msg)
  {
    n_msg = static_cast<int>(
        job.msg->corner_point_vec_x.size() + job.msg->t_point_vec_x.size() +
        job.msg->cross_point_vec_x.size());

    auto features =
        robocup_localization::inference::ObservationEncoder::fromMessage(*job.msg);
    features.pan_rad = job.pan_rad;

    // Corner-arc T junctions are in the MCL map but not in the training set.
    n_arc_dropped =
        robocup_localization::inference::ObservationEncoder::dropCornerArcT(
            features);

    const auto obs = inference_encoder_.buildObs(features, job.yaw_deg);

    n_valid = robocup_localization::inference::countValidSlots(obs);
    if (n_valid >= inference_config_.feature_min &&
        inference_config_.mlp_enable && inference_.mlpReady())
    {
      mlp = inference_.runMlp(obs);
      ran = true;
    }
    // Separate gate: ClsReg is trained on feature_min=1 and is most useful exactly
    // where the MLP is skipped (one landmark, ambiguous position).
    if (n_valid >= inference_config_.reloc_feature_min &&
        inference_config_.reloc_enable && inference_.relocReady())
    {
      reloc = inference_.runReloc(obs);
      ran = true;
    }
  }

  std::vector<robocup_localization::app::RelocProposal> reloc_proposals;
  if (reloc.valid)
  {
    reloc_proposals.reserve(reloc.candidates.size());
    for (const auto& candidate : reloc.candidates)
    {
      reloc_proposals.push_back({candidate.x_m, candidate.y_m});
    }
  }

  {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    inference_state_.mlp = mlp;
    inference_state_.reloc = std::move(reloc);
  }

  if (mlp.valid)
  {
    geometry_msgs::msg::PointStamped mlp_msg;
    mlp_msg.header.stamp = node->now();
    mlp_msg.header.frame_id = "field";
    mlp_msg.point.x = mlp.x_m;
    mlp_msg.point.y = mlp.y_m;
    mlp_pub_->publish(mlp_msg);
  }

  // Skipped frames reach here too so old samples expire on time; runs on the
  // inference thread so each estimate is counted exactly once.
  bool initial_state;
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    initial_state =
        game_state_.state == robocup_localization::app::STATE_INITIAL;
  }
  // MCL is paused in INITIAL, so disagreement with it means nothing.
  if (initial_state)
  {
    kidnap_.reset();
    reloc_recovery_.reset();
  }

  const double now_sec = node->get_clock()->now().seconds();

  if (kidnap_.config().enable && !initial_state)
  {
    float pf_x_px;
    float pf_y_px;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      pf_x_px = robot0.x;
      pf_y_px = robot0.y;
    }

    robocup_localization::app::KidnapFrame frame;
    frame.pf_x_m = FieldCoord::toFieldX(pf_x_px);
    frame.pf_y_m = FieldCoord::toFieldY(pf_y_px);
    frame.mlp_valid = mlp.valid;
    frame.mlp_x_m = mlp.x_m;
    frame.mlp_y_m = mlp.y_m;
    frame.yaw_deg = job.yaw_deg;
    frame.robot_y_sign = localization_engine_.robotYSign();
    if (mlp.valid && job.msg)
    {
      frame.observations = buildVisionPoints(
          *job.msg, pf_x_px, pf_y_px, static_cast<float>(job.yaw_deg));
    }

    const auto verdict = kidnap_.update(now_sec, std::move(frame));

    if (verdict.kidnapped)
    {
      applyKidnapVerdict(verdict);
      reloc_recovery_.holdOff(now_sec);
    }
    else if (verdict.rejected)
    {
      RCLCPP_WARN(
          node->get_logger(),
          "[kidnap] 기각: 후보 (%.2f, %.2f) m  점수 %.3f vs 현재 %.3f "
          "(필요: >= %.3f 이고 >= x%.2f)  frames=%d",
          verdict.x_m, verdict.y_m,
          verdict.candidate_score, verdict.current_score,
          kidnap_.config().verify_min_score, kidnap_.config().verify_ratio,
          verdict.frames);
    }
  }

  // Every frame goes in, blind ones too, so the window ages on wall time.
  if (reloc_recovery_.config().enable && !initial_state)
  {
    robocup_localization::app::RelocFrame frame;
    float pf_x_px;
    float pf_y_px;
    {
      std::lock_guard<std::mutex> lock(robot0_mutex);
      pf_x_px = robot0.x;
      pf_y_px = robot0.y;
      frame.odom_x_m = odom_x_m_;
      frame.odom_y_m = odom_y_m_;
    }
    frame.pf_x_m = FieldCoord::toFieldX(pf_x_px);
    frame.pf_y_m = FieldCoord::toFieldY(pf_y_px);
    frame.yaw_deg = job.yaw_deg;
    frame.robot_y_sign = localization_engine_.robotYSign();
    if (job.has_features && job.msg)
    {
      frame.observations = buildVisionPoints(
          *job.msg, pf_x_px, pf_y_px, static_cast<float>(job.yaw_deg));
    }
    frame.proposals = std::move(reloc_proposals);

    const auto verdict = reloc_recovery_.update(now_sec, std::move(frame));
    if (verdict.relocalize)
    {
      const auto [old_x_m, old_y_m] = teleportRobot(verdict.x_m, verdict.y_m);
      kidnap_.reset();
      RCLCPP_WARN(
          node->get_logger(),
          "[reloc] (%.2f, %.2f) -> (%.2f, %.2f) m  점수 %.3f vs 현재 %.3f  후보 %d개",
          old_x_m, old_y_m, verdict.x_m, verdict.y_m,
          verdict.candidate_score, verdict.current_score, verdict.proposals);
    }
    else if (verdict.lost)
    {
      RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "[reloc] 위치 불일치(현재 %.3f < %.2f), 채택할 후보 없음: 최고 %.3f (후보 %d개)",
          verdict.current_score, reloc_recovery_.config().lost_fit,
          verdict.candidate_score, verdict.proposals);
    }
  }

  if (!ran)
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 3000,
        "[inference] 건너뜀: msg 특징점 %d개 -> 아크T 제외 %d개 "
        "-> 게이트 통과 %d개 (feature_min=%d)",
        n_msg, n_arc_dropped, n_valid, inference_config_.feature_min);
    return;
  }

  const std::chrono::duration<double, std::milli> elapsed =
      std::chrono::steady_clock::now() - started;
  unsigned long dropped;
  {
    std::lock_guard<std::mutex> lock(inference_job_mutex_);
    dropped = inference_dropped_;
  }
  RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "[inference] %.2f ms, drop=%lu", elapsed.count(), dropped);
}

std::pair<double, double> QNode::teleportRobot(double x_m, double y_m)
{
  const float x_px = static_cast<float>(FieldCoord::toPixelX(x_m));
  const float y_px = static_cast<float>(FieldCoord::toPixelY(y_m));

  float yaw_deg;
  float old_x;
  float old_y;
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    old_x = robot0.x;
    old_y = robot0.y;
    robot0.x = x_px;
    robot0.y = y_px;
    robot0.trail.clear();
    yaw_deg = robot0.yaw_deg;  // heading comes from the IMU; the models give position only
  }

  // Otherwise particles left at the old pose pull the estimate back.
  resetParticles(x_px, y_px, yaw_deg, particle_range_);
  return {FieldCoord::toFieldX(old_x), FieldCoord::toFieldY(old_y)};
}

void QNode::applyKidnapVerdict(
    const robocup_localization::app::KidnapVerdict& verdict)
{
  const auto [old_x_m, old_y_m] = teleportRobot(verdict.x_m, verdict.y_m);
  RCLCPP_WARN(
      node->get_logger(),
      "[kidnap] (%.2f, %.2f) -> (%.2f, %.2f) m  frames=%d spread=%.2fm  "
      "점수 %.3f vs 현재 %.3f",
      old_x_m, old_y_m,
      verdict.x_m, verdict.y_m, verdict.frames, verdict.spread_m,
      verdict.candidate_score, verdict.current_score);
}

void QNode::previewRobotPosition(float x, float y)
{
  {
    std::lock_guard<std::mutex> lock(robot0_mutex);
    robot0.x = x;
    robot0.y = y;
    robot0.trail.clear();
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
    robot0.trail.clear();
    yaw_deg = robot0.yaw_deg;
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
      robot_yaw_deg = robot0.yaw_deg;
    }

    const auto result =
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
  rclcpp::WallRate loop_rate(localization_config_.loop_rate.ros_spin_hz);
  auto last_update = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    // spin_some takes one message per subscription, which falls behind 50 Hz odometry at this rate.
    rclcpp::spin_all(node, std::chrono::milliseconds(10));

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
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const bool compensate = latency_compensation_ && stamp_ns > 0;
  std::int64_t age_ns = 0;
  if (compensate)
  {
    age_ns = node->now().nanoseconds() - stamp_ns;
  }

  float robot0_x;
  float robot0_y;
  float robot0_z;
  robocup_localization::core::YawLookup yaw_lookup =
      robocup_localization::core::YawLookup::Empty;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    robot0_x = robot0.x;
    robot0_y = robot0.y;
    robot0_z = robot0.yaw_deg;

    // The features were seen at capture time; rotating them with the current yaw smears them while turning.
    if (compensate)
    {
      double capture_yaw_deg = 0.0;
      yaw_lookup = yaw_history_.lookup(stamp_ns, capture_yaw_deg);
      if (yaw_lookup == robocup_localization::core::YawLookup::Ok)
      {
        robot0_z = static_cast<float>(capture_yaw_deg);
      }
    }
  }

  if (compensate &&
      (age_ns > max_feature_age_ns_ ||
       yaw_lookup == robocup_localization::core::YawLookup::TooOld))
  {
    RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "[latency] vision_feature %.0f ms old (max %.0f ms), dropped",
        age_ns / 1e6, max_feature_age_ns_ / 1e6);
    return;
  }

  std::vector<VISION_POINT> observations =
      buildVisionPoints(*msg, robot0_x, robot0_y, robot0_z);

  bool mcl_active;
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    mcl_active =
        game_state_.state != robocup_localization::app::STATE_INITIAL;
  }
  const bool has_features = !observations.empty();
  const bool valid_observation =
      has_features && vision_pan_flag_ == 0 && mcl_active;

  localization_engine_.submitFeatures(
      std::move(observations), valid_observation, robot0_z);

  // Own thread, independent of the MCL gates, so inference never stalls spin or publishing.
  if (localization_config_.inference.mlp || localization_config_.inference.reloc)
  {
    {
      std::lock_guard<std::mutex> lock(inference_job_mutex_);
      if (inference_pending_)
      {
        // Still busy: overwrite with the newest frame, since stale poses are useless.
        ++inference_dropped_;
      }
      inference_job_ =
          InferenceJob{msg, robot0_z, vision_pan_rad_.load(), has_features};
      inference_pending_ = true;
    }
    inference_cv_.notify_one();
  }

  if (valid_observation)
  {
    {
      std::lock_guard<std::mutex> lock(mcl_mutex_);
      mcl_pending_ = true;
    }
    mcl_cv_.notify_one();
  }
}

void QNode::imuCallback(
    const humanoid_interfaces::msg::ImuMsg::SharedPtr msg,
    const rclcpp::MessageInfo& info)
{
  // ImuMsg has no header; the DDS publish time is exact, while callback time lags by a spin period.
  std::int64_t time_ns = info.get_rmw_message_info().source_timestamp;
  if (time_ns <= 0)
  {
    time_ns = node->now().nanoseconds();
  }

  std::lock_guard<std::mutex> lock(robot0_mutex);
  robot0.yaw_deg = msg->yaw;
  yaw_history_.push(time_ns, msg->yaw);
}

void QNode::visionCallback(const vision_interfaces::msg::Robocupvision::SharedPtr msg)
{
  localization_engine_.setBallObservation(
      msg->ball_2d_x, msg->ball_2d_y);
  vision_pan_flag_ = msg->flag_pan;

  // Vision sends no detected flag; the refiner zeroes distance and position when the ball is lost.
  const bool ball_detected =
      msg->ball_d > 0.0 && (msg->ball_2d_x != 0.0 || msg->ball_2d_y != 0.0);

  vision_pan_rad_ = msg->pan;

  if (ball_detected)
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
      robot0_z = robot0.yaw_deg;
    }
    robot_y_sign =
        PreFilter::computeYSign(robot0_z, goal_post_pan_deg_buf_);

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
    robot_pose.yaw_deg = robot0.yaw_deg;
  }

  robocup_localization::core::VisionObservation observation;
  observation.ball_detected = ball_detected;
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
  const double ik_Xmoved = msg->x;
  const double ik_Ymoved = msg->y;

  double ik_dx;
  double ik_dy;
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    const double ik_prev_x = robot0.x;
    const double ik_prev_y = robot0.y;

    robot0.move(ik_Xmoved, ik_Ymoved);

    ik_dx = robot0.x - ik_prev_x;
    ik_dy = robot0.y - ik_prev_y;
    // Map y grows downward; field meters for the reloc track.
    odom_x_m_ += ik_dx / FieldCoord::PX_PER_M_X;
    odom_y_m_ -= ik_dy / FieldCoord::PX_PER_M_Y;
  }

  const double ik_trans = std::sqrt(ik_dx * ik_dx + ik_dy * ik_dy);

  robocup_localization::core::ParticleMotion motion;
  motion.translation_px = ik_trans;

  if (ik_trans >= 0.1)
  {
    const double ik_rot1 = std::atan2(ik_Ymoved, ik_Xmoved);
    motion.world_direction_rad = std::atan2(ik_dy, ik_dx);
    motion.translation_stddev = ROBOT::translationStddev(ik_trans, ik_rot1);
    motion.direction_stddev = ROBOT::directionStddev(ik_trans, ik_rot1);
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
    robot0.trail.clear();
    robot_x = robot0.x;
    robot_y = robot0.y;
    robot_yaw_deg = robot0.yaw_deg;
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
