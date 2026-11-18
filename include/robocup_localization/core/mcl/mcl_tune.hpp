#pragma once

#include "robot_config.h"

namespace MclTune
{

#if defined(FIELD_SMALL)
  constexpr double OBS_FILTER_DIST_M           = 5.0;
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 2.0;
  constexpr double L_DIST_GATE_M               = 0.5;
  constexpr double MIN_JUMP_M                  = 0.07;
  constexpr double MAX_JUMP_M                  = 3.0;
  constexpr double MAX_SPREAD_M                = 0.5;

#elif defined(FIELD_MIDDLE)
  constexpr double OBS_FILTER_DIST_M           = 2.64;
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 3.11;
  constexpr double L_DIST_GATE_M               = 0.78;
  constexpr double MIN_JUMP_M                  = 0.11;
  constexpr double MAX_JUMP_M                  = 1.56;
  constexpr double MAX_SPREAD_M                = 0.78;

#elif defined(FIELD_LARGE)
  constexpr double OBS_FILTER_DIST_M           = 3.56;
  constexpr double OBS_FILTER_X_FEATURE_DIST_M = 3.89;
  constexpr double L_DIST_GATE_M               = 1.22;
  constexpr double MIN_JUMP_M                  = 0.80;
  constexpr double MAX_JUMP_M                  = 2.54;
  constexpr double MAX_SPREAD_M                = 1.22;

#else
  #error "필드 타입 미정의. robot_config.h 를 확인하세요."
#endif

  constexpr double CIRCLE_MARGIN_M         = 1.5;
  constexpr double T_SHAPE_CIRCLE_MARGIN_M = 1.5;

}
