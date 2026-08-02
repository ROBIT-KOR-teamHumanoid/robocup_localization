#include "monte_carlo_localization/motion_model.hpp"

#include <cmath>

#include "field_profile/field_coordinate.hpp"
#include "monte_carlo_localization/mcl_engine.hpp"  // gaussianRandom (RNG 스트림 공유)

namespace
{
// 파티클을 필드 안으로 클램프. 필드 규격이 런타임이라 상수가 아니라 함수다.
inline double pxMinX() { return FieldCoord::toPixelX(-FieldCoord::A / 2); }
inline double pxMaxX() { return FieldCoord::toPixelX(FieldCoord::A / 2); }
inline double pxMinY() { return FieldCoord::toPixelY(FieldCoord::B / 2); }
inline double pxMaxY() { return FieldCoord::toPixelY(-FieldCoord::B / 2); }

inline void wrapYaw(float &z)
{
  if (z > 180.0f)
    z -= 360.0f;
  if (z < -180.0f)
    z += 360.0f;
}
}  // namespace

MotionModel::Delta MotionModel::applyOdometry(SelfRobot &robot0, double ik_x,
                                              double ik_y,
                                              bool foot_align) const
{
  Delta d;

  const double prev_x = robot0.x;
  const double prev_y = robot0.y;

  robot0.recordTrail(ik_x, ik_y);
  robot0.last_ik_cmd_x = ik_x;
  robot0.last_ik_cmd_y = ik_y;

  // 전/후진, 좌/우 이동의 실제 보폭이 달라 부호별로 다른 배율을 쓴다.
  const double sx =
      ik_x * ((ik_x > 0) ? config.odom_scale_forward : config.odom_scale_backward);
  const double sy =
      ik_y * ((ik_y > 0) ? config.odom_scale_left : config.odom_scale_right);

  // 로봇 기준 이동을 현재 헤딩으로 world-frame 에 얹는다.
  const double heading_rad = (robot0.yaw_deg + 90) * M_PI / 180.0;
  robot0.x += sx * std::cos(heading_rad) - sy * std::sin(heading_rad);
  robot0.y -= sx * std::sin(heading_rad) + sy * std::cos(heading_rad);

  // 필드 밖으로 나가지 않게 자른다. 델타는 자른 뒤 값으로 계산해야
  // 파티클이 로봇보다 더 움직이는 일이 없다.
  if (robot0.x < pxMinX()) robot0.x = pxMinX();
  if (robot0.x > pxMaxX()) robot0.x = pxMaxX();
  if (robot0.y < pxMinY()) robot0.y = pxMinY();
  if (robot0.y > pxMaxY()) robot0.y = pxMaxY();

  d.dx = robot0.x - prev_x;
  d.dy = robot0.y - prev_y;

  if (foot_align)
  {
    d.dx *= config.foot_align_odom_scale;
    d.dy *= config.foot_align_odom_scale;
  }

  d.trans = std::sqrt(d.dx * d.dx + d.dy * d.dy);
  d.rot1 = std::atan2(ik_y, ik_x);

  return d;
}

void MotionModel::propagate(std::vector<Particle> &pt, int particle_num,
                            const Delta &d, double d_yaw) const
{
  // 제자리: 위치는 지터만, 회전은 그대로 반영
  if (d.trans < STATIONARY_TRANS_PX)
  {
    for (int i = 0; i < particle_num; i++)
    {
      if (config.noise_enable)
      {
        pt[i].x += MclEngine::gaussianRandom(0, STATIONARY_JITTER_PX);
        pt[i].y += MclEngine::gaussianRandom(0, STATIONARY_JITTER_PX);
      }
      pt[i].yaw_deg += d_yaw;
      wrapYaw(pt[i].yaw_deg);
    }
    return;
  }

  const double world_dir = std::atan2(d.dy, d.dx);

  for (int i = 0; i < particle_num; i++)
  {
    if (config.noise_enable)
    {
      const double trans_dev = ALPHA3 * d.trans + ALPHA4 * std::fabs(d.rot1);
      const double rot1_dev = ALPHA1 * std::fabs(d.rot1) + ALPHA2 * d.trans;

      const double n_trans = d.trans + MclEngine::gaussianRandom(0, trans_dev);
      const double n_dir = world_dir + MclEngine::gaussianRandom(0, rot1_dev);

      pt[i].x += n_trans * std::cos(n_dir);
      pt[i].y += n_trans * std::sin(n_dir);

      const double yaw_noise =
          MclEngine::gaussianRandom(0, ALPHA5 * std::fabs(d_yaw) + 0.1);
      pt[i].yaw_deg += d_yaw + yaw_noise;
    }
    else
    {
      pt[i].x += d.trans * std::cos(world_dir);
      pt[i].y += d.trans * std::sin(world_dir);
      pt[i].yaw_deg += d_yaw;
    }

    wrapYaw(pt[i].yaw_deg);

    if (pt[i].x < pxMinX())
      pt[i].x = pxMinX();
    if (pt[i].x > pxMaxX())
      pt[i].x = pxMaxX();
    if (pt[i].y < pxMinY())
      pt[i].y = pxMinY();
    if (pt[i].y > pxMaxY())
      pt[i].y = pxMaxY();
  }
}
