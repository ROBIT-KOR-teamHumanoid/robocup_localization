#pragma once

#include <mutex>
#include <vector>

#include "objects_model/ball.hpp"
#include "objects_model/obstacle.hpp"
#include "objects_model/self_robot.hpp"
#include "objects_model/teammate.hpp"

// 공·장애물 추적
// ============================================================
// 공 위치는 콜백으로 받는 값이 아니라 **융합 결과**다.
//   자기 비전(로봇 기준 mm) + 팀 로봇이 UDP 로 보낸 공 + 신뢰 박스 + 타임아웃
// 을 합쳐 하나의 필드 좌표를 만든다. 장애물도 마찬가지로 비전의 상대 좌표를
// 필드 좌표로 올리고 미검출 카운터로 만료시킨다.
//
// 예전엔 이 계산이 MainWindow::select_ball() 에 있었다. GUI 요소가 하나도
// 없는 순수 도메인 로직이라 여기로 옮겼다.
//
// update() 는 ROS 타이머 스레드가, 화면 표시는 Qt 스레드가 읽는다.
// MclNode 와 같은 방식으로 snapshot() 이 한 번에 복사해 일관된 시점을 준다.
class BallTracker
{
public:
  // visionMSG 에서 이 계산에 필요한 것만 추린 입력.
  // QNode 의 VisionMsg 를 직접 받지 않는 이유는 헤더 순환을 피하기 위해서다.
  struct VisionInput
  {
    float ball_2d_x_mm = 0.0f;  // 로봇 기준 공 위치
    float ball_2d_y_mm = 0.0f;
    float ball_d_mm = 0.0f;     // 공까지 거리 (0 이면 미검출)
    float ball_speed_x = 0.0f;  // 로봇 기준 공 속도
    float ball_speed_y = 0.0f;
    std::vector<float> robot_vec_x;  // 감지된 다른 로봇들의 상대 좌표 (mm)
    std::vector<float> robot_vec_y;
  };

  // 관측 시점의 자기 pose (맵 픽셀 + 도)
  struct Pose
  {
    float x_px = 0.0f;
    float y_px = 0.0f;
    float yaw_deg = 0.0f;
  };

  /// 한 프레임 갱신: 팀 데이터 만료 처리 → 장애물 좌표 → 자기 공 → 팀 공과
  /// 비교해 최종 선택 → 필드 경계 클램프.
  /// 팀 로봇 pose 는 QNode 소유라 참조로 받는다(신선도 decay 로 갱신된다).
  void update(const VisionInput &vision, const Pose &robot_pose,
              Teammate &robot1, Teammate &robot2, Teammate &robot3,
              Teammate &robot4);

  // 화면 표시용 사본. 여러 값을 따로 읽으면 그 사이 갱신되어 공 위치와
  // 장애물이 서로 다른 시점의 조합이 될 수 있다.
  struct Snapshot
  {
    BALL ball;
    std::vector<OBSTACLE> obstacles;
    std::vector<OBSTACLE> debug_obstacles;
  };
  Snapshot snapshot();

  // GUI 조작 (공 수동 지정/삭제, 디버그 장애물). 락을 잡고 처리한다.
  void setBall(float x_px, float y_px);

  /// GUI 수동 배치 상태를 반영한다. manual_active 면 set_x/set_y 로 고정하고,
  /// 아니면서 비전이 오래 끊겼으면 공을 지운다. 매 프레임 호출.
  void applyManualPlacement(bool manual_active, bool vision_stale);
  void clearBall();
  void addDebugObstacle(float x_px, float y_px);
  void clearDebugObstacles();
  void popDebugObstacle();       // 마지막 하나 제거
  void clearTeammateBalls();     // 팀 로봇이 보고한 공 전부 초기화

  std::mutex mutex_;  // 아래 상태 전체를 보호

  // ── 추적 결과 (소유) ──────────────────────────────────────────
  BALL ball;                             // 최종 선택된 공
  BALL ball1, ball2, ball3, ball4;       // 팀 로봇이 보고한 공
  std::vector<OBSTACLE> obstacles;       // 비전이 감지한 로봇들
  std::vector<OBSTACLE> debug_obstacles; // GUI 에서 수동 배치한 것

  // 장애물 미검출 카운터 상한. 이 값에 도달하면 좌표를 0 으로 지운다.
  static constexpr int OBSTACLE_LOST_MAX = 300;
  // 공 미검출 카운터 상한 (약 3초 @ 20Hz).
  static constexpr int BALL_LOST_MAX = 60;
};
