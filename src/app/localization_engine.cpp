#include "robocup_localization/app/localization_engine.hpp"

#include <mutex>
#include <utility>

namespace robocup_localization::app
{

LocalizationEngine::LocalizationEngine(core::WorldModelConfig config)
    : world_model_(config)
{
}

void LocalizationEngine::configureParticleFilter(
    const core::ParticleFilterConfig& config)
{
  particle_filter_.configure(config);
}

void LocalizationEngine::setParticleRange(float range)
{
  particle_filter_.setParticleRange(range);
}

void LocalizationEngine::resetParticles(
    float x, float y, float yaw_deg, float range)
{
  particle_filter_.randomize(x, y, yaw_deg, range);
}

void LocalizationEngine::clearParticleFilterVision()
{
  particle_filter_.clearVisionPoints();
}

void LocalizationEngine::submitFeatures(
    std::vector<VISION_POINT> observations,
    bool valid_observation,
    double robot_yaw_deg)
{
  particle_filter_.submitFeatures(
      std::move(observations), valid_observation, robot_yaw_deg);
}

core::MclResult LocalizationEngine::correctParticles(
    float robot_x, float robot_y, float robot_yaw_deg)
{
  return particle_filter_.correct(robot_x, robot_y, robot_yaw_deg);
}

void LocalizationEngine::setBallObservation(
    double relative_x_mm, double relative_y_mm)
{
  particle_filter_.setBallObservation(relative_x_mm, relative_y_mm);
}

void LocalizationEngine::mirrorParticlesY(float map_height)
{
  particle_filter_.mirrorY(map_height);
}

void LocalizationEngine::setRobotYSign(int sign)
{
  particle_filter_.setRobotYSign(sign);
}

int LocalizationEngine::robotYSign() const
{
  return particle_filter_.robotYSign();
}

void LocalizationEngine::applyParticleMotion(
    const core::ParticleMotion& motion)
{
  particle_filter_.applyMotion(motion);
}

core::ParticleFilterSnapshot LocalizationEngine::particleFilterSnapshot() const
{
  return particle_filter_.snapshot();
}

void LocalizationEngine::updateVision(
    const core::Pose2D& robot_pose,
    const core::VisionObservation& observation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.updateVision(robot_pose, observation);
}

void LocalizationEngine::updateTeammate(
    std::size_t index,
    const core::TeammateObservation& observation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.updateTeammate(index, observation);
}

void LocalizationEngine::setManualBall(float x, float y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.setManualBall(x, y);
}

void LocalizationEngine::clearManualBall()
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.clearManualBall();
}

void LocalizationEngine::clearTeammates()
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.clearTeammates();
}

void LocalizationEngine::addManualObstacle(float x, float y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.addManualObstacle(x, y);
}

void LocalizationEngine::removeLastManualObstacle()
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.removeLastManualObstacle();
}

void LocalizationEngine::update(double dt_seconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  world_model_.update(dt_seconds);
}

core::WorldModelSnapshot LocalizationEngine::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return world_model_.snapshot();
}

}
