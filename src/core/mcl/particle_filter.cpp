#include "robocup_localization/core/mcl/particle_filter.hpp"
#include "robocup_localization/core/mcl/random_util.hpp"

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

void ParticleFilter::setBallObservation(
    double relative_x_mm, double relative_y_mm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.ball_relative_x_mm = relative_x_mm;
  pipeline_.ball_relative_y_mm = relative_y_mm;
}

void ParticleFilter::setRobotYSign(int sign)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.robot_y_sign = sign;
}

int ParticleFilter::robotYSign() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pipeline_.robot_y_sign;
}

void ParticleFilter::submitFeatures(
    std::vector<VISION_POINT> observations,
    bool valid_observation,
    double robot_yaw_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!valid_observation)
  {
    pipeline_.valid_observation = false;
    pipeline_.vision_point_vect.clear();
    pipeline_.vision_data_count = 0;
    pipeline_.vision_frame_count = 0;
    return;
  }

  pipeline_.vision_data_count +=
      static_cast<int>(observations.size());
  pipeline_.vision_frame_count += 1;
  pipeline_.vision_point_vect.insert(
      pipeline_.vision_point_vect.end(),
      std::make_move_iterator(observations.begin()),
      std::make_move_iterator(observations.end()));
  // No setCircle here: run() recomputes it, and the UI draws CIRCLE_* directly,
  // so updating it mid-accumulation makes the circle jump.
  pipeline_.valid_observation = true;
}

MclResult ParticleFilter::correct(
    float robot_x, float robot_y, float robot_yaw_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  MclInput input;
  input.robot_x = robot_x;
  input.robot_y = robot_y;
  input.robot_yaw_deg = robot_yaw_deg;
  input.particle_range = config_.particle_range;
  input.min_fit = config_.min_fit;
  input.noise_enabled = config_.noise_enabled;
  input.threads = config_.threads;

  // Heading is not estimated: every particle takes the IMU yaw, so only position is filtered.
  for (auto& particle : particles_)
  {
    particle.yaw_deg = robot_yaw_deg;
  }

  return pipeline_.run(particles_, input);
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
        particle.x += gaussianRandom(0.0, 0.1);
        particle.y += gaussianRandom(0.0, 0.1);
      }
    }
    else if (config_.noise_enabled)
    {
      const double translation = motion.translation_px +
          gaussianRandom(0.0, motion.translation_stddev);
      const double direction = motion.world_direction_rad +
          gaussianRandom(0.0, motion.direction_stddev);
      particle.x += translation * std::cos(direction);
      particle.y += translation * std::sin(direction);
    }
    else
    {
      particle.x +=
          motion.translation_px * std::cos(motion.world_direction_rad);
      particle.y +=
          motion.translation_px * std::sin(motion.world_direction_rad);
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
    randomizeParticle(particle, x, y, range, yaw_deg);
  }
  // The last weights describe particles that no longer exist.
  pipeline_.scored_particles.clear();
  pipeline_.scored_weights.clear();
}

void ParticleFilter::mirrorY(float map_height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& particle : particles_)
  {
    particle.y = map_height - particle.y;
    particle.yaw_deg = 180.0f - particle.yaw_deg;
    if (particle.yaw_deg > 180.0f)
    {
      particle.yaw_deg -= 360.0f;
    }
    if (particle.yaw_deg < -180.0f)
    {
      particle.yaw_deg += 360.0f;
    }
  }
}

void ParticleFilter::clearVisionPoints()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.vision_point_vect.clear();
  pipeline_.vision_data_count = 0;
  pipeline_.vision_frame_count = 0;
  pipeline_.valid_observation = false;
}

ParticleFilterSnapshot ParticleFilter::snapshot() const
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  std::lock_guard<std::mutex> cache_lock(snapshot_cache_mutex_);
  if (lock.owns_lock())
  {
    snapshot_cache_.particles = particles_;
    snapshot_cache_.vision_points = pipeline_.vision_point_vect;
    snapshot_cache_.filter = pipeline_.filter;
    snapshot_cache_.scored_particles = pipeline_.scored_particles;
    snapshot_cache_.scored_weights = pipeline_.scored_weights;
  }
  return snapshot_cache_;
}

std::size_t ParticleFilter::particleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return particles_.size();
}

}
