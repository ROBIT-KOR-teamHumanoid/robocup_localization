#pragma once

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
// 락 없음 — 현재는 GUI 타이머 스레드에서만 update() 가 불린다. 발행을 ROS
// 타이머로 옮기려면 그때 락이 필요하다.
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
