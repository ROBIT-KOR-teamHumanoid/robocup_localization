#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include "robocup_localization/core/mcl/line_feature.hpp"
#include "robocup_localization/core/types/particle.hpp"
#include "robocup_localization/core/types/vision_point.hpp"
#include "robocup_localization/core/mcl/observation_filter.hpp"

namespace robocup_localization::core
{

struct ParticleFilterConfig
{
  std::size_t particle_count = 8000;
  double weight_update_threshold = 14.0;
  float particle_range = 0.0f;
  bool noise_enabled = true;
};

struct ParticleMotion
{
  double translation_px = 0.0;
  double world_direction_rad = 0.0;
  double translation_stddev = 0.0;
  double direction_stddev = 0.0;
  double yaw_delta_deg = 0.0;
  double yaw_stddev = 0.0;
};

struct ParticleFilterSnapshot
{
  std::vector<Particle> particles;
  std::vector<VISION_POINT> vision_points;
  Filter filter;
};

class ParticleFilter
{
public:
  explicit ParticleFilter(ParticleFilterConfig config = {});

  void configure(const ParticleFilterConfig& config);
  void setParticleRange(float particle_range);
  void setObservationFilterDistance(double distance_m);
  void setBallObservation(double relative_x_mm, double relative_y_mm);
  void setRobotYSign(int sign);
  int robotYSign() const;

  void submitFeatures(
      std::vector<VISION_POINT> observations,
      bool valid_observation,
      double robot_yaw_deg);

  FeatureCalcResult correct(
      float robot_x, float robot_y, float robot_yaw_deg);

  void applyMotion(const ParticleMotion& motion);
  void randomize(float x, float y, float yaw_deg, float range);
  void mirrorY(float map_height);
  void clearVisionPoints();

  ParticleFilterSnapshot snapshot() const;
  std::size_t particleCount() const;

private:
  mutable std::mutex mutex_;
  ParticleFilterConfig config_;
  Line_Feature likelihood_;
  std::vector<Particle> particles_;
};

}  // namespace robocup_localization::core
