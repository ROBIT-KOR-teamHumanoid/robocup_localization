#include "monte_carlo_localization/particle_reset.hpp"

#include "field_profile/field_coordinate.hpp"
#include "monte_carlo_localization/mcl_engine.hpp"  // gaussianRandom / random_point

namespace ParticleReset
{

void mirrorAboutMidline(std::vector<Particle> &pt, int particle_num)
{
  const float mirror = static_cast<float>(FieldCoord::MIRROR_Y_PX);

  for (int i = 0; i < particle_num; i++)
  {
    pt[i].y = mirror - pt[i].y;

    // 위치만 뒤집으면 헤딩이 관측과 어긋난다. y 반전은 yaw 를 180-yaw 로 만든다.
    double new_yaw = 180.0 - pt[i].yaw_deg;
    if (new_yaw > 180.0)
      new_yaw -= 360.0;
    if (new_yaw < -180.0)
      new_yaw += 360.0;
    pt[i].yaw_deg = static_cast<float>(new_yaw);
  }
}

void scatterAround(std::vector<Particle> &pt, int particle_num,
                   float x_px, float y_px, float yaw_deg, float range_px)
{
  for (int i = 0; i < particle_num; i++)
    MclEngine::random_point(pt[i], x_px, y_px, range_px, yaw_deg);
}

}  // namespace ParticleReset
