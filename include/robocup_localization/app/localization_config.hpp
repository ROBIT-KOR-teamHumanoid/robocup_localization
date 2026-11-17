#pragma once

namespace robocup_localization::app
{

  struct InferenceEnableConfig
  {
    bool mlp = false;
    bool reloc = false;
  };

  struct LoopRateConfig
  {
    double ros_spin_hz = 20.0;
    double ui_data_hz = 20.0;
    double ui_render_hz = 5.0;
  };

  struct InitialPose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;
  };

  // Same values as robocup_task_planner state_pose.yaml (left side only); the right side
  // is the 180-degree rotation, as in StatePoseConfig.resolve_for_side.
  struct InitialPoseConfig
  {
    static constexpr int ROBOT_COUNT = 4;

    InitialPose left[ROBOT_COUNT];

    // my_side 0 = left (Gamecontroldata.myside).
    InitialPose forSide(int robotnum, int my_side) const
    {
      if (robotnum < 1 || robotnum > ROBOT_COUNT)
      {
        return InitialPose();
      }

      InitialPose pose = left[robotnum - 1];
      if (my_side == 0)
      {
        return pose;
      }

      double yaw = pose.yaw_deg + 180.0;
      while (yaw > 180.0)
      {
        yaw -= 360.0;
      }
      while (yaw <= -180.0)
      {
        yaw += 360.0;
      }

      InitialPose mirrored;
      mirrored.x = -pose.x;
      mirrored.y = -pose.y;
      mirrored.yaw_deg = yaw;
      return mirrored;
    }
  };

  struct LocalizationConfig
  {
    LoopRateConfig loop_rate;
    InferenceEnableConfig inference;
    InitialPoseConfig initial_pose;
  };

}
