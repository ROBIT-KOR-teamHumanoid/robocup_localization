#pragma once

// ============================================================
// Team UDP Receiver
//   - master 를 경유해 받은 다른 팀 로봇의 위치/공 데이터를 내부 ROBOT/BALL 에 반영
//   - 좌표 단위는 픽셀 (팀 전체 공유 와이어 포맷)
// ============================================================

#include "humanoid_interfaces/msg/udp2master.hpp"

#include "../robot_odom_model/robot_odom_model.hpp"
#include "../objects/ball.hpp"

namespace team_udp_receiver
{
// UDP 로 받은 한 팀 로봇의 상태(위치/야우/상태/공)를 robot, ball 에 반영한다.
// ball.d 는 해당 로봇에서 본 공까지의 픽셀 거리 (가장 가까운 공 선택에 사용)
void receive(const humanoid_interfaces::msg::Udp2master::SharedPtr &msg,
             ROBOT &robot, BALL &ball);

// 매 프레임 호출: 신선도(robot.b) decay, 만료 시(b<=0) 로봇/공 비활성화
void tick(ROBOT &robot, BALL &ball, float decay = 0.005f);


bool selectNearestBall(BALL &own, double own_ball_dist_px,
                       const BALL &b1, const BALL &b2, const BALL &b3, const BALL &b4);
}
