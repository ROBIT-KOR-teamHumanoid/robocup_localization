#include "robocup_localization/core/mcl/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

#include "field_coordinate/field_coordinate.hpp"

namespace robocup_localization::core
{

ParticleFilter::ParticleFilter(ParticleFilterConfig config)
    : config_(config),
      particles_(config.particle_count)
{
}

void ParticleFilter::configure(const ParticleFilterConfig& config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  particles_.resize(config_.particle_count);
}

void ParticleFilter::setParticleRange(float particle_range)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.particle_range = particle_range;
}

void ParticleFilter::setObservationFilterDistance(double distance_m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  likelihood_.filter.OBS_FILTER_DIST_M = distance_m;
}

void ParticleFilter::setBallObservation(
    double relative_x_mm, double relative_y_mm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  likelihood_.ball_relative_x_mm = relative_x_mm;
  likelihood_.ball_relative_y_mm = relative_y_mm;
}

void ParticleFilter::setRobotYSign(int sign)
{
  std::lock_guard<std::mutex> lock(mutex_);
  likelihood_.robot_y_sign = sign;
}

int ParticleFilter::robotYSign() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return likelihood_.robot_y_sign;
}

void ParticleFilter::submitFeatures(
    std::vector<VISION_POINT> observations,
    bool valid_observation,
    double robot_yaw_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!valid_observation)
  {
    likelihood_.valid_observation = false;
    likelihood_.vision_point_vect.clear();
    likelihood_.vision_data_count = 0;
    likelihood_.vision_data_size = 0;
    return;
  }

  likelihood_.vision_data_count +=
      static_cast<int>(observations.size());
  likelihood_.vision_data_size +=
      static_cast<int>(observations.size());
  likelihood_.vision_point_vect.insert(
      likelihood_.vision_point_vect.end(),
      std::make_move_iterator(observations.begin()),
      std::make_move_iterator(observations.end()));
  likelihood_.filter.setCircle(
      robot_yaw_deg, likelihood_.vision_point_vect);
  likelihood_.valid_observation = true;
}

FeatureCalcResult ParticleFilter::correct(
    float robot_x, float robot_y, float robot_yaw_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  FeatureCalcInput input;
  input.robot_x = robot_x;
  input.robot_y = robot_y;
  input.robot_yaw_deg = robot_yaw_deg;
  input.particle_range = config_.particle_range;
  input.weight_update_threshold = config_.weight_update_threshold;
  input.noise_enabled = config_.noise_enabled;

  return likelihood_.featureCalc(particles_, input);
}

void ParticleFilter::applyMotion(const ParticleMotion& motion)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto& particle : particles_)
  {
    if (motion.translation_px < 0.1)
    {
      if (config_.noise_enabled)
      {
        particle.x += Line_Feature::gaussianRandom(0.0, 0.1);
        particle.y += Line_Feature::gaussianRandom(0.0, 0.1);
      }
      particle.z += motion.yaw_delta_deg;
    }
    else if (config_.noise_enabled)
    {
      const double translation = motion.translation_px +
          Line_Feature::gaussianRandom(0.0, motion.translation_stddev);
      const double direction = motion.world_direction_rad +
          Line_Feature::gaussianRandom(0.0, motion.direction_stddev);
      particle.x += translation * std::cos(direction);
      particle.y += translation * std::sin(direction);
      particle.z += motion.yaw_delta_deg +
          Line_Feature::gaussianRandom(0.0, motion.yaw_stddev);
    }
    else
    {
      particle.x +=
          motion.translation_px * std::cos(motion.world_direction_rad);
      particle.y +=
          motion.translation_px * std::sin(motion.world_direction_rad);
      particle.z += motion.yaw_delta_deg;
    }

    while (particle.z > 180.0f)
    {
      particle.z -= 360.0f;
    }
    while (particle.z < -180.0f)
    {
      particle.z += 360.0f;
    }

    particle.x = std::clamp(
        particle.x,
        static_cast<float>(FieldCoord::F_X_MIN),
        static_cast<float>(FieldCoord::F_X_MAX));
    particle.y = std::clamp(
        particle.y,
        static_cast<float>(FieldCoord::F_Y_MIN),
        static_cast<float>(FieldCoord::F_Y_MAX));
  }
}

void ParticleFilter::randomize(
    float x, float y, float yaw_deg, float range)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_.particle_range = range;
  for (auto& particle : particles_)
  {
    Line_Feature::random_point(particle, x, y, range, yaw_deg);
  }
}

void ParticleFilter::mirrorY(float map_height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& particle : particles_)
  {
    particle.y = map_height - particle.y;
    particle.z = 180.0f - particle.z;
    if (particle.z > 180.0f)
    {
      particle.z -= 360.0f;
    }
    if (particle.z < -180.0f)
    {
      particle.z += 360.0f;
    }
  }
}

void ParticleFilter::clearVisionPoints()
{
  std::lock_guard<std::mutex> lock(mutex_);
  likelihood_.vision_point_vect.clear();
  likelihood_.vision_data_count = 0;
  likelihood_.vision_data_size = 0;
  likelihood_.valid_observation = false;
}

ParticleFilterSnapshot ParticleFilter::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ParticleFilterSnapshot result;
  result.particles = particles_;
  result.vision_points = likelihood_.vision_point_vect;
  result.filter = likelihood_.filter;
  return result;
}

std::size_t ParticleFilter::particleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return particles_.size();
}

}  // namespace robocup_localization::core
