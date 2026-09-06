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

  /// 로봇 한 대의 INITIAL 배치. 필드 좌표 미터, yaw 는 도.
  struct InitialPose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;
  };

  /// robotnum 1~4 의 INITIAL 배치.
  ///
  /// 값은 robocup_task_planner 의 state_pose.yaml 과 같은 것을 쓴다 — 심판이
  /// 로봇을 내려놓는 자리는 하나이므로, 추정기와 판단기가 서로 다른 자리를
  /// 알고 있으면 곤란하다. 런치가 그 yaml 을 읽어 이 파라미터로 넘긴다.
  ///
  /// yaml 은 왼쪽 진영만 적는다. 오른쪽 진영이면 필드 중앙을 축으로 180도
  /// 돌려 쓴다 (x, y 부호 반전 + yaw 에 180). task_planner 의
  /// StatePoseConfig.resolve_for_side 와 같은 변환이다.
  struct InitialPoseConfig
  {
    static constexpr int ROBOT_COUNT = 4;

    InitialPose left[ROBOT_COUNT];

    /// robotnum(1~4) 의 배치를 현재 진영 기준으로 돌려준다.
    /// my_side 가 0 이면 왼쪽, 아니면 오른쪽이다 (Gamecontroldata.myside).
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

} // namespace robocup_localization::app
