#pragma once

namespace robocup_localization::app
{

class LocalizationCommand
{
public:
  virtual ~LocalizationCommand() = default;

  virtual void previewRobotPosition(float x, float y) = 0;
  virtual void placeRobot(float x, float y, float particle_range) = 0;
  virtual void setRobotOdometryCalibration(
      float forward, float backward, float left, float right) = 0;
  virtual void setManualBall(float x, float y) = 0;
  virtual void clearManualBall() = 0;
  virtual void clearTeammates() = 0;
  virtual void addManualObstacle(float x, float y) = 0;
  virtual void removeLastManualObstacle() = 0;
  virtual void setParticleRange(float range) = 0;
  virtual void clearParticleFilterVision() = 0;
};

}  // namespace robocup_localization::app
