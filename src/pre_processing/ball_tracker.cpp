#include "pre_processing/ball_tracker.hpp"

#include <cmath>

#include "field_profile/field_coordinate.hpp"
#include "pre_processing/vision_processing.hpp"
#include "utils/team_udp_receiver.hpp"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180)
#endif

namespace
{
// 필드 경계로 자르되, 0 은 "미검출" 표식이라 건드리지 않는다.
template <typename T>
void clampX(T &v)
{
  if (v < FieldCoord::F_X_MIN && v != 0)
    v = FieldCoord::F_X_MIN;
  if (v > FieldCoord::F_X_MAX)
    v = FieldCoord::F_X_MAX;
}

template <typename T>
void clampY(T &v)
{
  if (v < FieldCoord::F_Y_MIN && v != 0)
    v = FieldCoord::F_Y_MIN;
  if (v > FieldCoord::F_Y_MAX)
    v = FieldCoord::F_Y_MAX;
}
}  // namespace

void BallTracker::update(const VisionInput &vision, const Pose &robot_pose,
                         Teammate &robot1, Teammate &robot2,
                         Teammate &robot3, Teammate &robot4)
{
  // ── 1. 팀 데이터 신선도 decay, 만료 시 비활성화 ────────────────
  team_udp_receiver::tick(robot1, ball1);
  team_udp_receiver::tick(robot2, ball2);
  team_udp_receiver::tick(robot3, ball3);
  team_udp_receiver::tick(robot4, ball4);

  // ── 2. 장애물: 상대 mm → 필드 픽셀, 안 보이면 카운터 증가 ──────
  const size_t seen = vision.robot_vec_x.size();
  if (obstacles.size() < seen)
    obstacles.resize(seen);

  for (size_t i = 0; i < obstacles.size(); i++)
  {
    if (i < seen)
    {
      const auto px = VisionProcessing::toFieldPx(
          vision.robot_vec_x[i], vision.robot_vec_y[i],
          {robot_pose.x_px, robot_pose.y_px, robot_pose.yaw_deg});
      obstacles[i].x = px.x;
      obstacles[i].y = px.y;
      obstacles[i].nocnt = 0;
    }
    else
    {
      obstacles[i].nocnt += 1;
      if (obstacles[i].nocnt > OBSTACLE_LOST_MAX)
      {
        obstacles[i].nocnt = OBSTACLE_LOST_MAX;
        obstacles[i].x = 0;
        obstacles[i].y = 0;
      }
    }
  }

  // ── 3. 자기 비전 공 → 필드 좌표 ───────────────────────────────
  if (vision.ball_2d_x_mm != 0 || vision.ball_2d_y_mm != 0)
  {
    const auto ball_px = VisionProcessing::toFieldPx(
        vision.ball_2d_x_mm, vision.ball_2d_y_mm,
        {robot_pose.x_px, robot_pose.y_px, robot_pose.yaw_deg});
    const int computed_x = static_cast<int>(ball_px.x);
    const int computed_y = static_cast<int>(ball_px.y);

    // 필드 가장자리에서 계산된 공은 오차가 커서 신뢰 박스 안쪽만 받는다.
    if (computed_x >= FieldCoord::BALL_TRUST_X_MIN &&
        computed_x <= FieldCoord::BALL_TRUST_X_MAX &&
        computed_y >= FieldCoord::BALL_TRUST_Y_MIN &&
        computed_y <= FieldCoord::BALL_TRUST_Y_MAX)
    {
      ball.noballcnt = 0;
      ball.x = computed_x;
      ball.y = computed_y;

      // 속도도 로봇 기준이라 필드 축으로 회전시킨다.
      const double c = std::cos((-1) * robot_pose.yaw_deg * DEG2RAD);
      const double s = std::sin((-1) * robot_pose.yaw_deg * DEG2RAD);
      ball.speed_x = vision.ball_speed_x * c - vision.ball_speed_y * s;
      ball.speed_y = vision.ball_speed_x * s + vision.ball_speed_y * c;
    }
    else
    {
      ball.noballcnt += 1;
      if (ball.noballcnt >= BALL_LOST_MAX)
        ball.noballcnt = BALL_LOST_MAX;
    }
  }
  else
  {
    // 아예 못 봤을 때
    ball.noballcnt += 1;
    if (ball.noballcnt >= BALL_LOST_MAX)
    {
      ball.x = 0;
      ball.y = 0;
      ball.noballcnt = BALL_LOST_MAX;
    }
  }

  // ── 4. 자기 관측 vs 팀 관측 중 가장 가까운 것 선택 ─────────────
  // 공을 봤어도 필드 밖으로 계산됐으면(noballcnt > 0) 신뢰하지 않는다.
  const double own_dist_px =
      (vision.ball_d_mm == 0 || ball.noballcnt > 0)
          ? 999999.0
          : vision.ball_d_mm * FieldCoord::MM2PX;

  team_udp_receiver::selectNearestBall(ball, own_dist_px, ball1, ball2, ball3,
                                       ball4);

  // ── 5. 필드 경계 클램프 ───────────────────────────────────────
  clampX(ball.x);
  clampY(ball.y);
  for (auto &obs : obstacles)
  {
    clampX(obs.x);
    clampY(obs.y);
  }
}
