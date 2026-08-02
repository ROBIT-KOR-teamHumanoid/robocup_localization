#pragma once

#include "field_profile/field_spec.hpp"

// 좌표계:
//   필드 좌표 - 원점: 필드 중앙, x: 오른쪽(+), y: 위쪽(+), 단위: 미터
//   픽셀 좌표 - 원점: 이미지 좌상단, x: 오른쪽(+), y: 아래쪽(+)

namespace FieldCoord
{

  constexpr int MAP_W = 1100; // 맵 이미지 너비
  constexpr int MAP_H = 800;  // 맵 이미지 높이

  // ── 입력: 필드 규격 ──────────────────────────────────────────────
  extern FieldSpecs::FieldSpec field_profile_params;

  inline const double &A = field_profile_params.A;             // 필드 길이
  inline const double &B = field_profile_params.B;             // 필드 너비
  inline const double &C = field_profile_params.C;             // 골 깊이
  inline const double &D = field_profile_params.D;             // 골 너비
  inline const double &E = field_profile_params.E;             // 골 에어리어 길이
  inline const double &F = field_profile_params.F;             // 골 에어리어 너비
  inline const double &G = field_profile_params.G;             // 페널티 에어리어 길이
  inline const double &H = field_profile_params.H;             // 페널티 에어리어 너비
  inline const double &I_DIST = field_profile_params.I_DIST;   // 페널티 마크 거리
  inline const double &J = field_profile_params.J;             // 센터 서클 지름
  inline const double &K = field_profile_params.K;             // 보더 스트립 너비
  inline const double &L_ARC = field_profile_params.L_ARC;     // 코너 아크 반지름
  inline const double &LINE_W = field_profile_params.LINE_W;   // 라인 너비
  inline const double &MARK_SZ = field_profile_params.MARK_SZ; // 페널티/센터 마크 크기

  // ── 특징점 타입 ──────────────────────────────────────────────────

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
    double x; // 필드 X 좌표
    double y; // 필드 Y 좌표
    FeatureType type;
    bool active = true;
  };

  //   ㄱ-shape 12 (index 0~11) + T-shape 10 (12~21) + X-shape 2 (22~23) + 골대 2 (24~25)
  constexpr int FEATURE_POINT_COUNT = 26;

  // 골대는 방향 기준(yaw_compensation, computeYSign)으로 따로 꺼내 쓴다.
  constexpr int IDX_GOALPOST_L = 24;
  constexpr int IDX_GOALPOST_R = 25;

  // ── 출력: 규격에서 계산된 파생값 ─────────────────────────────────
  struct FieldDerived
  {
    double world_w, world_h;
    double px_per_m;                 // 미터 to 픽셀
    double origin_x_px, origin_y_px; // 레터박스 오프셋 (월드를 맵 중앙에 배치)
    double mirror_y_px;              // y=0 기준 픽셀 플립 축
    double mm2px;                    // mm to 맵 픽셀

    int circle_margin_px;         // 파티클 필터 서클 마진 (1.5m)
    int t_shape_circle_margin_px; // T자 특징점 감지 시 추가 마진 (1.5m)

    int f_x_min, f_x_max, f_y_min, f_y_max; // 필드 경계 픽셀

    // 필드 경계 근처에서 계산된 공은 오차가 커서 신뢰 박스를 안쪽으로 들여놓는다.
    int ball_edge_margin_px;
    int ball_trust_x_min, ball_trust_x_max, ball_trust_y_min, ball_trust_y_max;

    FeaturePoint all_points[FEATURE_POINT_COUNT];
  };

  extern FieldDerived field_profile_derived;

  inline const double &PX_PER_M = field_profile_derived.px_per_m;
  inline const double &ORIGIN_X_PX = field_profile_derived.origin_x_px;
  inline const double &ORIGIN_Y_PX = field_profile_derived.origin_y_px;
  inline const double &MIRROR_Y_PX = field_profile_derived.mirror_y_px;
  inline const double &MM2PX = field_profile_derived.mm2px;

  inline const int &CIRCLE_MARGIN_PX = field_profile_derived.circle_margin_px;
  inline const int &T_SHAPE_CIRCLE_MARGIN_PX =
      field_profile_derived.t_shape_circle_margin_px;

  inline const int &F_X_MIN = field_profile_derived.f_x_min; // 필드 왼쪽 경계
  inline const int &F_X_MAX = field_profile_derived.f_x_max; // 필드 오른쪽 경계
  inline const int &F_Y_MIN = field_profile_derived.f_y_min; // 필드 위쪽 경계
  inline const int &F_Y_MAX = field_profile_derived.f_y_max; // 필드 아래쪽 경계

  inline const int &BALL_TRUST_X_MIN = field_profile_derived.ball_trust_x_min;
  inline const int &BALL_TRUST_X_MAX = field_profile_derived.ball_trust_x_max;
  inline const int &BALL_TRUST_Y_MIN = field_profile_derived.ball_trust_y_min;
  inline const int &BALL_TRUST_Y_MAX = field_profile_derived.ball_trust_y_max;

  inline const FeaturePoint (&ALL_POINTS)[FEATURE_POINT_COUNT] =
      field_profile_derived.all_points;

  // ── 좌표 변환 ────────────────────────────────────────────────────

  constexpr int roundPx(double v) { return static_cast<int>(v + 0.5); }

  /// 필드 미터 좌표 -> 이미지 픽셀 X
  inline int toPixelX(double fx)
  {
    return roundPx(ORIGIN_X_PX + (fx + A * 0.5 + K) * PX_PER_M);
  }

  /// 필드 미터 좌표 -> 이미지 픽셀 Y
  inline int toPixelY(double fy)
  {
    return roundPx(ORIGIN_Y_PX + (B * 0.5 + K - fy) * PX_PER_M);
  }

  /// 이미지 픽셀 X -> 필드 미터 좌표
  inline double toFieldX(double px)
  {
    return (px - ORIGIN_X_PX) / PX_PER_M - A * 0.5 - K;
  }

  /// 이미지 픽셀 Y -> 필드 미터 좌표  (Y축 반전)
  inline double toFieldY(double py)
  {
    return B * 0.5 + K - (py - ORIGIN_Y_PX) / PX_PER_M;
  }

  // ── 초기화 ───────────────────────────────────────────────────────
  void init(const FieldSpecs::FieldSpec &spec);
  bool initialized();

} // namespace FieldCoord
