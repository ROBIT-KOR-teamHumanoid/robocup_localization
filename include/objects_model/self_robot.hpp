#pragma once

#include <ctime>
#include <vector>

#include "objects_model/pose2d.hpp"

// 자기 로봇
// ============================================================
// pose 에 더해 "이 로봇에만 의미 있는" 것들을 갖는다. 팀 로봇(Teammate)과
// 한 타입이던 시절엔 robot0.freshness 처럼 의미 없는 접근이 컴파일됐다.
struct SelfRobot : Pose2D
{
  // 화면에 그리는 이동 자취 한 점
  struct TrailPoint
  {
    unsigned int time_sec;  // time(NULL) 값
    int x;                  // 맵 픽셀
    int y;
  };

  TrailPoint trail_point;
  std::vector<TrailPoint> trail;  // 최근 10점만 유지

  // 마지막으로 받은 ik 이동 명령 (UI 표시용)
  float last_ik_cmd_x = 0.0f;
  float last_ik_cmd_y = 0.0f;

  // 이동 명령이 있으면 현재 위치를 자취에 남기고, 정지하면 자취를 비운다.
  void recordTrail(double ik_x, double ik_y)
  {
    if (ik_x == 0 && ik_y == 0)
    {
      trail.clear();
      return;
    }

    trail_point.time_sec = time(NULL);
    trail_point.x = static_cast<int>(x);
    trail_point.y = static_cast<int>(y);
    trail.push_back(trail_point);
    if (trail.size() > 10)
      trail.erase(trail.begin());
  }
};
