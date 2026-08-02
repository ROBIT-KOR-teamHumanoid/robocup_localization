#pragma once

#include "objects_model/pose2d.hpp"

// 팀 로봇 (UDP 로 받는 다른 로봇)
// ============================================================
// pose 에 더해 수신 상태를 갖는다. 자기 로봇에는 의미가 없는 값들이라
// SelfRobot 과 타입을 나눴다.
struct Teammate : Pose2D
{
  // 데이터 신선도. UDP 수신 시 1.0, 매 프레임 감쇠, 0 이면 만료로 보고
  // pose 와 공 정보를 지운다 (utils/team_udp_receiver).
  float freshness = 0.0f;

  // 동작 상태 (UDP robotcase). 화면 색으로 표시된다.
  int state = 0;
};
