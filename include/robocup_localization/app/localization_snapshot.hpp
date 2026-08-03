#pragma once

#include <string>
#include <vector>

#include "robocup_localization/core/types/pose2d.hpp"
#include "robocup_localization/app/localization_config.hpp"
#include "robocup_localization/core/world_model.hpp"
#include "robocup_localization/core/mcl/particle_filter.hpp"

namespace robocup_localization::app
{

struct RobotTrailPoint
{
  int X = 0;
  int Y = 0;
};

struct RobotStateSnapshot
{
  core::Pose2D pose;
  float odom_forward = 0.0f;
  float odom_backward = 0.0f;
  float odom_left = 0.0f;
  float odom_right = 0.0f;
  float ik_x = 0.0f;
  float ik_y = 0.0f;
  std::vector<RobotTrailPoint> trail;
};

struct MasterStateSnapshot
{
  int master_target_x = 0;
  int master_target_y = 0;
  float master_best_position_attack_x = 0.0f;
  float master_best_position_attack_y = 0.0f;
  float master_best_position_defend_x = 0.0f;
  float master_best_position_defend_y = 0.0f;
  float master_angle_to_target = 0.0f;
  std::string master_move2ball_phase;
};

struct GameStateSnapshot
{
  int mySide = 1;
  int penalty = 0;
  int position = 0;
  int robotNum = 0;
  int state = 0;
  int ready_count = 0;
};

struct LocalizationSnapshot
{
  core::WorldModelSnapshot world;
  RobotStateSnapshot robot;
  core::ParticleFilterSnapshot particle_filter;
  MasterStateSnapshot master;
  GameStateSnapshot game;
};

class LocalizationSnapshotSource
{
public:
  virtual ~LocalizationSnapshotSource() = default;
  virtual LocalizationSnapshot localizationSnapshot() const = 0;
  virtual LocalizationConfig localizationConfig() const noexcept = 0;
};

}  // namespace robocup_localization::app
