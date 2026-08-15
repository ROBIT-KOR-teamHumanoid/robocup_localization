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

void ParticleFilter::setObservationFilterDistance(double distance_m)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_.filter.OBS_FILTER_DIST_M = distance_m;
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
    pipeline_.vision_data_size = 0;
    return;
  }

  pipeline_.vision_data_count +=
      static_cast<int>(observations.size());
  pipeline_.vision_data_size +=
      static_cast<int>(observations.size());
  pipeline_.vision_point_vect.insert(
      pipeline_.vision_point_vect.end(),
      std::make_move_iterator(observations.begin()),
      std::make_move_iterator(observations.end()));
  // 여기서 setCircle 을 부르지 않는다.
  //   · MCL 계산에는 불필요하다 — MclPipeline::run 이 dropUnmatched/가중치 계산
  //     직전에 setCircle 을 다시 부른다(같은 누적 버퍼로).
  //   · CIRCLE_CENTER/CIRCLE_R 은 UI 가 그대로 그리는 멤버라(main_window.cpp),
  //     한 사이클에 여러 번 다른 값으로 덮어쓰면 화면의 원이 튄다. 이 호출은
  //     **아직 correct 가 돌지 않은 누적 중간 상태**로 원을 그리게 만들었다.
  //   남은 writer 는 두 개다: checkLocalPoint(부분집합, start=synthetic_cnt) 와
  //   run() 의 setCircle(전체). 둘도 서로 다른 원이라 UI 는 여전히 흔들릴 수 있다.
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
  input.weight_update_threshold = config_.weight_update_threshold;
  input.noise_enabled = config_.noise_enabled;

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
      particle.yaw_deg += motion.yaw_delta_deg;
    }
    else if (config_.noise_enabled)
    {
      const double translation = motion.translation_px +
          gaussianRandom(0.0, motion.translation_stddev);
      const double direction = motion.world_direction_rad +
          gaussianRandom(0.0, motion.direction_stddev);
      particle.x += translation * std::cos(direction);
      particle.y += translation * std::sin(direction);
      particle.yaw_deg += motion.yaw_delta_deg +
          gaussianRandom(0.0, motion.yaw_stddev);
    }
    else
    {
      particle.x +=
          motion.translation_px * std::cos(motion.world_direction_rad);
      particle.y +=
          motion.translation_px * std::sin(motion.world_direction_rad);
      particle.yaw_deg += motion.yaw_delta_deg;
    }

    while (particle.yaw_deg > 180.0f)
    {
      particle.yaw_deg -= 360.0f;
    }
    while (particle.yaw_deg < -180.0f)
    {
      particle.yaw_deg += 360.0f;
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
  pipeline_.vision_data_size = 0;
  pipeline_.valid_observation = false;
}

ParticleFilterSnapshot ParticleFilter::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ParticleFilterSnapshot result;
  result.particles = particles_;
  result.vision_points = pipeline_.vision_point_vect;
  result.filter = pipeline_.filter;
  return result;
}

std::size_t ParticleFilter::particleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return particles_.size();
}

}  // namespace robocup_localization::core
