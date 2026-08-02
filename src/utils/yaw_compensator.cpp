#include "utils/yaw_compensator.hpp"

#include <cmath>

#include "field_profile/field_coordinate.hpp"

namespace YawCompensator
{

int computeYSign(double robot_yaw_deg,
                 const std::vector<double> &goal_post_pan_degs)
{
  if (goal_post_pan_degs.empty())
    return 0;

  // PAN_POSITION = -servo_right_deg → published pan 은 오른쪽이 음수
  // vec_y = -d*cos(yaw+pan)*PX = ry_m*PX → cos(yaw+pan) < 0 이면 ry_m > 0
  // |cos| < COS_DEAD_ZONE 구간 (yaw+pan ≈ ±90도)은 판별 불가 → skip

  int pos_votes = 0;
  int neg_votes = 0;
  for (double pan_deg : goal_post_pan_degs)
  {
    const double world_angle_rad = (robot_yaw_deg + pan_deg) * M_PI / 180.0;
    const double c = std::cos(world_angle_rad);
    if (std::fabs(c) < COS_DEAD_ZONE)
      continue;
    if (c < 0.0)
      pos_votes++;
    else
      neg_votes++;
  }

  if (pos_votes > neg_votes)
    return 1;
  if (neg_votes > pos_votes)
    return -1;
  return 0;
}

Result estimate(double robot_x_m, double robot_y_m, double current_yaw_deg,
                const std::vector<double> &goal_post_pan_degs)
{
  Result out;
  if (goal_post_pan_degs.empty())
    return out;

  const FieldCoord::FeaturePoint goal_posts[2] = {
      FieldCoord::ALL_POINTS[FieldCoord::IDX_GOALPOST_L],
      FieldCoord::ALL_POINTS[FieldCoord::IDX_GOALPOST_R]};

  double min_diff = 1e9;
  double best_yaw = 0.0;

  for (double pan_deg : goal_post_pan_degs)
  {
    for (int g = 0; g < 2; g++)
    {
      // 현재 위치에서 이 골대를 정면으로 보려면 필요한 yaw
      const double expected_yaw =
          std::atan2(goal_posts[g].x - robot_x_m, goal_posts[g].y - robot_y_m) *
          180.0 / M_PI;

      double candidate_yaw = expected_yaw - pan_deg;
      while (candidate_yaw > 180.0)
        candidate_yaw -= 360.0;
      while (candidate_yaw < -180.0)
        candidate_yaw += 360.0;

      double diff = std::fabs(candidate_yaw - current_yaw_deg);
      if (diff > 180.0)
        diff = 360.0 - diff;

      if (diff < min_diff)
      {
        min_diff = diff;
        best_yaw = candidate_yaw;
      }
    }
  }

  out.valid = true;
  out.corrected_yaw_deg = best_yaw;
  out.drift_deg = min_diff;
  return out;
}

}  // namespace YawCompensator
