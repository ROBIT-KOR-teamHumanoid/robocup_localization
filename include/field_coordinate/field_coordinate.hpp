#pragma once

#include "robot_config.h"


// Field frame: origin at center, +x right, +y up (m). Pixel frame: top-left origin, +y down.
// Exactly one of FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE must be defined in robot_config.h.
namespace FieldCoord
{
#if defined(FIELD_SMALL)
  constexpr double A = 9.0;
  constexpr double B = 6.0;
  constexpr double C = 0.7;
  constexpr double D = 1.8;
  constexpr double E = 1.0;
  constexpr double F = 3.0;
  constexpr double G = 2.0;
  constexpr double H = 4.0;
  constexpr double I_DIST = 1.5;
  constexpr double J = 1.5;
  constexpr double K = 1.0;
  constexpr double L_ARC = 0.0;
  constexpr double LINE_W = 0.05;
  constexpr double MARK_SZ = 0.10;

#elif defined(FIELD_MIDDLE)
  constexpr double A = 14.0;
  constexpr double B = 9.0;
  constexpr double C = 1.2;
  constexpr double D = 2.6;
  constexpr double E = 1.0;
  constexpr double F = 4.0;
  constexpr double G = 3.0;
  constexpr double H = 6.0;
  constexpr double I_DIST = 2.0;
  constexpr double J = 3.0;
  constexpr double K = 1.0;
  constexpr double L_ARC = 0.5;
  constexpr double LINE_W = 0.09;
  constexpr double MARK_SZ = 0.10;

#elif defined(FIELD_LARGE)
  constexpr double A = 22.0;
  constexpr double B = 14.0;
  constexpr double C = 1.0;
  constexpr double D = 2.6;
  constexpr double E = 1.0;
  constexpr double F = 5.0;
  constexpr double G = 3.5;
  constexpr double H = 7.0;
  constexpr double I_DIST = 2.5;
  constexpr double J = 4.0;
  constexpr double K = 1.0;
  constexpr double L_ARC = 1.0;
  constexpr double LINE_W = 0.12;
  constexpr double MARK_SZ = 0.15;

#else
#error "필드 타입 미정의. robot_config.h 에 FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE 중 하나를 정의하세요."
#endif

  constexpr int MAP_W = 1100;
  constexpr int MAP_H = 800;

  constexpr double WORLD_W = A + 2.0 * K;
  constexpr double WORLD_H = B + 2.0 * K;

  constexpr double PX_PER_M_X = MAP_W / WORLD_W;
  constexpr double PX_PER_M_Y = MAP_H / WORLD_H;

  constexpr int toPixelX(double fx)
  {
    return static_cast<int>((fx + A * 0.5 + K) * PX_PER_M_X);
  }

  constexpr int toPixelY(double fy)
  {
    return static_cast<int>((B * 0.5 + K - fy) * PX_PER_M_Y);
  }

  constexpr double toFieldX(double px)
  {
    return px / PX_PER_M_X - A * 0.5 - K;
  }

  constexpr double toFieldY(double py)
  {
    return B * 0.5 + K - py / PX_PER_M_Y;
  }

  constexpr double MM2PX = PX_PER_M_X / 1000.0;

  constexpr int F_X_MIN = toPixelX(-A / 2);
  constexpr int F_X_MAX = toPixelX(A / 2);
  constexpr int F_Y_MIN = toPixelY(B / 2);
  constexpr int F_Y_MAX = toPixelY(-B / 2);

  enum FeatureType
  {
    L_SHAPE,
    T_SHAPE,
    X_SHAPE,
    PENALTY_MARK,
    GOAL_POST
  };

  struct FeaturePoint
  {
    double x;
    double y;
    FeatureType type;
    bool active = true;  // false: drawn only, excluded from weighting
  };

  constexpr FeaturePoint L00 = {-A / 2, B / 2, L_SHAPE, true};
  constexpr FeaturePoint L01 = {A / 2, B / 2, L_SHAPE, true};
  constexpr FeaturePoint L02 = {-A / 2, -B / 2, L_SHAPE, true};
  constexpr FeaturePoint L03 = {A / 2, -B / 2, L_SHAPE, true};
  constexpr FeaturePoint L04 = {-A / 2 + G, H / 2, L_SHAPE, true};
  constexpr FeaturePoint L05 = {-A / 2 + G, -H / 2, L_SHAPE, true};
  constexpr FeaturePoint L06 = {A / 2 - G, H / 2, L_SHAPE, true};
  constexpr FeaturePoint L07 = {A / 2 - G, -H / 2, L_SHAPE, true};
  constexpr FeaturePoint L08 = {-A / 2 + E, F / 2, L_SHAPE, true};
  constexpr FeaturePoint L09 = {-A / 2 + E, -F / 2, L_SHAPE, true};
  constexpr FeaturePoint L10 = {A / 2 - E, F / 2, L_SHAPE, true};
  constexpr FeaturePoint L11 = {A / 2 - E, -F / 2, L_SHAPE, true};

  constexpr FeaturePoint T00 = {0, B / 2, T_SHAPE, true};
  constexpr FeaturePoint T01 = {0, -B / 2, T_SHAPE, true};
  constexpr FeaturePoint T02 = {-A / 2, H / 2, T_SHAPE, true};
  constexpr FeaturePoint T03 = {-A / 2, -H / 2, T_SHAPE, true};
  constexpr FeaturePoint T04 = {-A / 2, F / 2, T_SHAPE, true};
  constexpr FeaturePoint T05 = {-A / 2, -F / 2, T_SHAPE, true};
  constexpr FeaturePoint T06 = {A / 2, H / 2, T_SHAPE, true};
  constexpr FeaturePoint T07 = {A / 2, -H / 2, T_SHAPE, true};
  constexpr FeaturePoint T08 = {A / 2, F / 2, T_SHAPE, true};
  constexpr FeaturePoint T09 = {A / 2, -F / 2, T_SHAPE, true};

  constexpr bool HAS_CORNER_ARC = L_ARC > 0.0;
  constexpr FeaturePoint T10 = {-A / 2 + L_ARC, B / 2, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T11 = {A / 2 - L_ARC, B / 2, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T12 = {-A / 2 + L_ARC, -B / 2, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T13 = {A / 2 - L_ARC, -B / 2, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T14 = {-A / 2, B / 2 - L_ARC, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T15 = {A / 2, B / 2 - L_ARC, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T16 = {-A / 2, -B / 2 + L_ARC, T_SHAPE, HAS_CORNER_ARC};
  constexpr FeaturePoint T17 = {A / 2, -B / 2 + L_ARC, T_SHAPE, HAS_CORNER_ARC};

  constexpr FeaturePoint X01 = {0, J / 2, X_SHAPE, true};
  constexpr FeaturePoint X02 = {0, -J / 2, X_SHAPE, true};

  constexpr FeaturePoint PM0 = {-A / 2 + I_DIST, 0, X_SHAPE, false};
  constexpr FeaturePoint PM1 = {A / 2 - I_DIST, 0, X_SHAPE, false};

  constexpr FeaturePoint Left_Goalpost = {-A / 2, 0, GOAL_POST, true};
  constexpr FeaturePoint Right_Goalpost = {A / 2, 0, GOAL_POST, true};
  // Index layout: L 0-11, T 12-21, corner-arc T 22-29 (inactive when L_ARC == 0),
  // X 30-31, penalty marks 32-33, goalposts 34-35.
  constexpr FeaturePoint ALL_POINTS[36] = {
      L00, L01, L02, L03, L04, L05, L06, L07, L08, L09, L10, L11,
      T00, T01, T02, T03, T04, T05, T06, T07, T08, T09,
      T10, T11, T12, T13, T14, T15, T16, T17,
      X01, X02,
      PM0, PM1,
      Left_Goalpost, Right_Goalpost};

  constexpr int FEATURE_POINT_COUNT = 36;

}
