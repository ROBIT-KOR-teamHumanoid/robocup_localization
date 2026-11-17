#pragma once

#include <string>
#include <vector>

#include "robocup_localization/core/types/pose2d.hpp"
#include "robocup_localization/core/types/trajectory_point.hpp"
#include "robocup_localization/app/localization_config.hpp"
#include "robocup_localization/core/world_model.hpp"
#include "robocup_localization/core/mcl/particle_filter.hpp"
#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::app
{

struct RobotStateSnapshot
{
  core::Pose2D pose;
  float odom_forward = 0.0f;
  float odom_backward = 0.0f;
  float odom_left = 0.0f;
  float odom_right = 0.0f;
  bool odom_metric = false;
  float ik_x = 0.0f;
  float ik_y = 0.0f;
  std::vector<core::TrajectoryPoint> trail;
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

// Must match gamecontroller Gamecontroldata and robocup_task_planner game_state.py.
constexpr int STATE_INITIAL = 0;
constexpr int STATE_READY = 1;
constexpr int STATE_SET = 2;
constexpr int STATE_PLAYING = 3;
constexpr int STATE_FINISHED = 4;

// Matches `#define LEFT 0` in gamecontroller main_window.hpp.
constexpr int SIDE_LEFT = 0;
constexpr int SIDE_RIGHT = 1;

struct GameStateSnapshot
{
  int mySide = SIDE_RIGHT;
  int penalty = 0;
  int position = 0;
  int robotNum = 0;
  int state = STATE_INITIAL;
  int ready_count = 0;
};

struct InferenceStateSnapshot
{
  inference::MlpResult mlp;
  inference::RelocResult reloc;
};

struct LocalizationSnapshot
{
  core::WorldModelSnapshot world;
  RobotStateSnapshot robot;
  core::ParticleFilterSnapshot particle_filter;
  MasterStateSnapshot master;
  GameStateSnapshot game;
  InferenceStateSnapshot inference;
};

class LocalizationSnapshotSource
{
public:
  virtual ~LocalizationSnapshotSource() = default;
  virtual LocalizationSnapshot localizationSnapshot() const = 0;
  virtual LocalizationConfig localizationConfig() const noexcept = 0;
};

}
