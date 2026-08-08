#pragma once

#include "robot_config.h"  // FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE

namespace MclTune
{

#if defined(FIELD_SMALL)
  constexpr double OBS_FILTER_DIST_M           = 1.7;   // pre-filter 거리 임계
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 2.0;   // X-shape 전용
  constexpr double L_DIST_GATE_M               = 0.5;   // L-shape 거리 매칭 게이트
  constexpr double MIN_JUMP_M                  = 0.07;  // 위치 보정 최소 (진동 억제)
  constexpr double MAX_JUMP_M                  = 1.0;   // 위치 보정 최대
  constexpr double MAX_SPREAD_M                = 0.5;   // top-N 파티클 분산 허용

#elif defined(FIELD_MIDDLE)
  constexpr double OBS_FILTER_DIST_M           = 2.64;
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 3.11;
  constexpr double L_DIST_GATE_M               = 0.78;
  constexpr double MIN_JUMP_M                  = 0.11;
  constexpr double MAX_JUMP_M                  = 1.56;
  constexpr double MAX_SPREAD_M                = 0.78;

#elif defined(FIELD_LARGE)
  constexpr double OBS_FILTER_DIST_M           = 4.16;
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 4.89;
  constexpr double L_DIST_GATE_M               = 1.22;
  constexpr double MIN_JUMP_M                  = 0.17;
  constexpr double MAX_JUMP_M                  = 2.44;
  constexpr double MAX_SPREAD_M                = 1.22;

#else
  #error "필드 타입 미정의. robot_config.h 를 확인하세요."
#endif

  // 관측 외접원 마진. 필드 크기와 무관하게 1.5m.
  constexpr double CIRCLE_MARGIN_M         = 1.5;
  constexpr double T_SHAPE_CIRCLE_MARGIN_M = 1.5;

}  // namespace MclTune
