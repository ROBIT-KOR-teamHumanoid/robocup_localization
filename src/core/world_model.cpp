#include "robocup_localization/core/world_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "field_coordinate/field_coordinate.hpp"

namespace robocup_localization::core
{

WorldModel::WorldModel(WorldModelConfig config)
    : config_(config)
{
}

void WorldModel::updateVision(
    const Pose2D& robot_pose,
    const VisionObservation& observation)
{
  snapshot_.ball_prediction_level = std::clamp(
      observation.ball_prediction_level, 0, 150);

  const std::size_t obstacle_count = std::min(
      observation.obstacle_x_mm.size(),
      observation.obstacle_y_mm.size());

  if (snapshot_.obstacles.size() < obstacle_count)
  {
    snapshot_.obstacles.resize(obstacle_count);
    obstacle_age_seconds_.resize(obstacle_count, 0.0);
  }

  // 이번 관측에서 다시 확인되지 않은 기존 장애물은 비활성화
  for (auto& obstacle : snapshot_.obstacles)
  {
    obstacle.nocnt = 1;
  }

  constexpr double kDegToRad =
      3.14159265358979323846 / 180.0;

  const double yaw_rad = robot_pose.yaw_deg * kDegToRad;
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);

  for (std::size_t i = 0; i < obstacle_count; ++i)
  {
    const double relative_x =
        observation.obstacle_x_mm[i] * FieldCoord::MM2PX;
    const double relative_y =
        observation.obstacle_y_mm[i] * FieldCoord::MM2PX;

    double global_x =
        robot_pose.x -
        (relative_x * cos_yaw + relative_y * sin_yaw);

    double global_y =
        robot_pose.y +
        (relative_x * sin_yaw - relative_y * cos_yaw);

    global_x = std::clamp(
        global_x,
        static_cast<double>(FieldCoord::F_X_MIN),
        static_cast<double>(FieldCoord::F_X_MAX));

    global_y = std::clamp(
        global_y,
        static_cast<double>(FieldCoord::F_Y_MIN),
        static_cast<double>(FieldCoord::F_Y_MAX));

    snapshot_.obstacles[i].x = static_cast<float>(global_x);
    snapshot_.obstacles[i].y = static_cast<float>(global_y);
    snapshot_.obstacles[i].nocnt = 0;

    obstacle_age_seconds_[i] = 0.0;
  }

  local_ball_observed_ = false;

  if (!observation.ball_detected)
  {
    return;
  }

  const double relative_ball_x =
      observation.ball_x_mm * FieldCoord::MM2PX;
  const double relative_ball_y =
      observation.ball_y_mm * FieldCoord::MM2PX;

  const double global_ball_x =
      robot_pose.x -
      (relative_ball_x * cos_yaw + relative_ball_y * sin_yaw);

  const double global_ball_y =
      robot_pose.y +
      (relative_ball_x * sin_yaw - relative_ball_y * cos_yaw);

  const bool ball_inside_field =
      global_ball_x >= FieldCoord::F_X_MIN &&
      global_ball_x <= FieldCoord::F_X_MAX &&
      global_ball_y >= FieldCoord::F_Y_MIN &&
      global_ball_y <= FieldCoord::F_Y_MAX;

  if (!ball_inside_field)
  {
    return;
  }

  local_ball_.x = static_cast<int>(global_ball_x);
  local_ball_.y = static_cast<int>(global_ball_y);
  local_ball_.d = static_cast<int>(
      observation.ball_distance_mm * FieldCoord::MM2PX);

  local_ball_.speed_x =
      observation.ball_speed_x * cos_yaw +
      observation.ball_speed_y * sin_yaw;

  local_ball_.speed_y =
      -observation.ball_speed_x * sin_yaw +
      observation.ball_speed_y * cos_yaw;

  local_ball_.noballcnt = 0;
  local_ball_.from_udp = false;

  local_ball_age_seconds_ = 0.0;
  local_ball_valid_ = true;
  local_ball_observed_ = true;
}

void WorldModel::updateTeammate(
    std::size_t index,
    const TeammateObservation& observation)
{
  if (index >= snapshot_.teammates.size())
  {
    return;
  }

  snapshot_.teammates[index] = observation;
  snapshot_.teammates[index].active = true;
  teammate_age_seconds_[index] = 0.0;
}

void WorldModel::setManualBall(float x, float y)
{
  manual_ball_ = BALL{};
  manual_ball_.x = static_cast<int>(std::clamp(
      static_cast<double>(x),
      static_cast<double>(FieldCoord::F_X_MIN),
      static_cast<double>(FieldCoord::F_X_MAX)));
  manual_ball_.y = static_cast<int>(std::clamp(
      static_cast<double>(y),
      static_cast<double>(FieldCoord::F_Y_MIN),
      static_cast<double>(FieldCoord::F_Y_MAX)));
  manual_ball_.noballcnt = 0;
  manual_ball_.from_udp = false;
  manual_ball_active_ = true;
  snapshot_.manual_ball_active = true;
}

void WorldModel::clearManualBall()
{
  manual_ball_ = BALL{};
  manual_ball_active_ = false;
  snapshot_.manual_ball_active = false;
}

void WorldModel::clearTeammates()
{
  snapshot_.teammates = {};
  teammate_age_seconds_ = {};
}

void WorldModel::addManualObstacle(float x, float y)
{
  OBSTACLE obstacle;
  obstacle.x = static_cast<float>(std::clamp(
      static_cast<double>(x),
      static_cast<double>(FieldCoord::F_X_MIN),
      static_cast<double>(FieldCoord::F_X_MAX)));
  obstacle.y = static_cast<float>(std::clamp(
      static_cast<double>(y),
      static_cast<double>(FieldCoord::F_Y_MIN),
      static_cast<double>(FieldCoord::F_Y_MAX)));
  obstacle.nocnt = 0;
  snapshot_.manual_obstacles.push_back(obstacle);
}

void WorldModel::removeLastManualObstacle()
{
  if (!snapshot_.manual_obstacles.empty())
  {
    snapshot_.manual_obstacles.pop_back();
  }
}

void WorldModel::update(double dt_seconds)
{
  expireTeammates(dt_seconds);
  expireObstacles(dt_seconds);
  selectBall(dt_seconds);
}

const WorldModelSnapshot& WorldModel::snapshot() const noexcept
{
  return snapshot_;
}

void WorldModel::expireTeammates(double dt_seconds)
{
  if (dt_seconds <= 0.0)
  {
    return;
  }

  for (std::size_t i = 0; i < snapshot_.teammates.size(); ++i)
  {
    auto& teammate = snapshot_.teammates[i];

    if (!teammate.active)
    {
      continue;
    }

    teammate_age_seconds_[i] += dt_seconds;

    if (teammate_age_seconds_[i] >= config_.teammate_timeout_sec)
    {
      teammate = TeammateObservation{};
      teammate_age_seconds_[i] = config_.teammate_timeout_sec;
    }
  }
}

void WorldModel::expireObstacles(double dt_seconds)
{
  if (dt_seconds <= 0.0)
  {
    return;
  }

  for (std::size_t i = 0; i < snapshot_.obstacles.size(); ++i)
  {
    obstacle_age_seconds_[i] += dt_seconds;

    if (obstacle_age_seconds_[i] >= config_.obstacle_timeout_sec)
    {
      snapshot_.obstacles[i] = OBSTACLE{};
      obstacle_age_seconds_[i] = config_.obstacle_timeout_sec;
    }
  }
}

void WorldModel::selectBall(double dt_seconds)
{
  if (dt_seconds > 0.0)
  {
    local_ball_age_seconds_ = std::min(
        local_ball_age_seconds_ + dt_seconds,
        config_.local_ball_lost_sec);
  }

  if (local_ball_valid_ &&
      local_ball_age_seconds_ >= config_.local_ball_lost_sec)
  {
    local_ball_ = BALL{};
    local_ball_valid_ = false;
  }

  if (local_ball_valid_)
  {
    snapshot_.ball = local_ball_;
    snapshot_.ball.noballcnt = local_ball_observed_ ? 0 : 1;
    snapshot_.ball.from_udp = false;
  }
  else
  {
    snapshot_.ball = BALL{};
  }

  const bool can_use_teammate_ball =
      local_ball_age_seconds_ > config_.teammate_ball_takeover_sec;

  const TeammateObservation* teammate =
      can_use_teammate_ball ? closestTeammateWithBall() : nullptr;

  if (teammate != nullptr)
  {
    BALL team_ball;
    team_ball.x = static_cast<int>(std::clamp(
        static_cast<double>(teammate->ball_x),
        static_cast<double>(FieldCoord::F_X_MIN),
        static_cast<double>(FieldCoord::F_X_MAX)));
    team_ball.y = static_cast<int>(std::clamp(
        static_cast<double>(teammate->ball_y),
        static_cast<double>(FieldCoord::F_Y_MIN),
        static_cast<double>(FieldCoord::F_Y_MAX)));
    team_ball.d = static_cast<int>(std::hypot(
        teammate->pose.x - teammate->ball_x,
        teammate->pose.y - teammate->ball_y));
    team_ball.noballcnt = 25;
    team_ball.from_udp = true;

    snapshot_.ball = team_ball;
  }

  if (manual_ball_active_)
  {
    snapshot_.ball = manual_ball_;
  }
}

const TeammateObservation* WorldModel::closestTeammateWithBall() const
{
  const TeammateObservation* closest = nullptr;
  double closest_distance = std::numeric_limits<double>::infinity();

  for (const auto& teammate : snapshot_.teammates)
  {
    if (!teammate.active ||
        (teammate.ball_x == 0.0f && teammate.ball_y == 0.0f))
    {
      continue;
    }

    const double distance = std::hypot(
        teammate.pose.x - teammate.ball_x,
        teammate.pose.y - teammate.ball_y);

    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest = &teammate;
    }
  }

  return closest;
}

}  // namespace robocup_localization::core