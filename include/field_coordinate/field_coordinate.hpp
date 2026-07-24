#pragma once

#include "robot_config.h"  // FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE 정의
//   robot_config.h 에서 아래 중 하나만 활성화
//
//     2026 RoboCup 규정을 기준으로 함
//     #define FIELD_SMALL    // S-Field: 9m × 6m  
//     #define FIELD_MIDDLE   // M-Field: 14m × 9m
//     #define FIELD_LARGE    // L-Field: 22m × 14m 
// 좌표계:
//   필드 좌표 - 원점: 필드 중앙, x: 오른쪽(+), y: 위쪽(+), 단위: 미터
//   픽셀 좌표 - 원점: 이미지 좌상단, x: 오른쪽(+), y: 아래쪽(+)


namespace FieldCoord {

// 필드 치수
// 범위가 있는 항목은 최댓값 사용

#if defined(FIELD_SMALL)
  // S-Field 
  constexpr double A      = 9.0;   // 필드 길이 (Field length)
  constexpr double B      = 6.0;   // 필드 너비 (Field width)
  constexpr double C      = 0.7;   // 골 깊이   (Goal depth)
  constexpr double D      = 1.8;   // 골 너비   (Goal width)
  constexpr double E      = 1.0;   // 골 에어리어 길이 (Goal area length)
  constexpr double F      = 3.0;   // 골 에어리어 너비 (Goal area width)
  constexpr double G      = 2.0;   // 페널티 에어리어 길이 (Penalty area length)
  constexpr double H      = 4.0;   // 페널티 에어리어 너비 (Penalty area width), 
  constexpr double I_DIST = 1.5;   // 페널티 마크 거리 (Penalty mark distance)
  constexpr double J      = 1.5;   // 센터 서클 지름 (Center circle diameter)
  constexpr double K      = 1.0;   // 보더 스트립 너비 (Border strip width, min)
  constexpr double L_ARC  = 0.0;   // 코너 아크 반지름 (Corner arc radius, none)
  constexpr double LINE_W = 0.05;  // 라인 너비 (Line width)
  constexpr double MARK_SZ= 0.10;  // 페널티/센터 마크 크기 (Mark size)

#elif defined(FIELD_MIDDLE)
  // M-Field 
  constexpr double A      = 14.0;
  constexpr double B      = 9.0;
  constexpr double C      = 1.2;   
  constexpr double D      = 2.6;   
  constexpr double E      = 1.0;
  constexpr double F      = 4.0;
  constexpr double G      = 3.0;
  constexpr double H      = 6.0;
  constexpr double I_DIST = 2.0;
  constexpr double J      = 3.0;
  constexpr double K      = 1.0;
  constexpr double L_ARC  = 0.5;
  constexpr double LINE_W = 0.09;
  constexpr double MARK_SZ= 0.10;

#elif defined(FIELD_LARGE)
  // L-Field
  constexpr double A      = 22.0;
  constexpr double B      = 14.0;
  constexpr double C      = 1.0;  
  constexpr double D      = 2.6;  
  constexpr double E      = 1.0;
  constexpr double F      = 5.0;
  constexpr double G      = 3.5;
  constexpr double H      = 7.0;
  constexpr double I_DIST = 2.5;
  constexpr double J      = 4.0;
  constexpr double K      = 1.0; 
  constexpr double L_ARC  = 1.0;
  constexpr double LINE_W = 0.12;
  constexpr double MARK_SZ= 0.15;

#else
  #error "필드 타입 미정의. robot_config.h 에 FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE 중 하나를 정의하세요."
#endif

// 맵 ui 파라미터
constexpr int    MAP_W    = 1100;          // 맵 이미지 너비
constexpr int    MAP_H    = 800;           // 맵 이미지 높이

constexpr double WORLD_W  = A + 2.0 * K;  // 표시 총 너비 
constexpr double WORLD_H  = B + 2.0 * K;  // 표시 총 높이 

constexpr double FIT_X    = MAP_W / WORLD_W;
constexpr double FIT_Y    = MAP_H / WORLD_H;
constexpr double PX_PER_M = (FIT_X < FIT_Y) ? FIT_X : FIT_Y;

// 레터박스 오프셋: 월드 영역을 맵 중앙에 배치
constexpr double ORIGIN_X_PX = (MAP_W - WORLD_W * PX_PER_M) * 0.5;
constexpr double ORIGIN_Y_PX = (MAP_H - WORLD_H * PX_PER_M) * 0.5;


// 좌표 변환 함수
// ============================================================

/// 맵 픽셀은 항상 양수이므로 +0.5 절삭으로 반올림한다.
constexpr int roundPx(double v) { return static_cast<int>(v + 0.5); }

/// 필드 미터 좌표 -> 이미지 픽셀 X
constexpr int toPixelX(double fx)
{
  return roundPx(ORIGIN_X_PX + (fx + A * 0.5 + K) * PX_PER_M);
}

/// 필드 미터 좌표 -> 이미지 픽셀 Y  (Y축 반전: 필드 위쪽 = 픽셀 위쪽)
constexpr int toPixelY(double fy)
{
  return roundPx(ORIGIN_Y_PX + (B * 0.5 + K - fy) * PX_PER_M);
}

/// 이미지 픽셀 X -> 필드 미터 좌표
constexpr double toFieldX(double px)
{
  return (px - ORIGIN_X_PX) / PX_PER_M - A * 0.5 - K;
}

/// 이미지 픽셀 Y -> 필드 미터 좌표  (Y축 반전)
constexpr double toFieldY(double py)
{
  return B * 0.5 + K - (py - ORIGIN_Y_PX) / PX_PER_M;
}

/// 레터박스가 중앙 정렬이므로 항상 MAP_H 와 같지만, 의도를 명시하기 위해 유도식으로 둔다.
constexpr double MIRROR_Y_PX = 2.0 * ORIGIN_Y_PX + WORLD_H * PX_PER_M;

// 파티클 필터 서클 마진 (0.5m to 픽셀)
constexpr int CIRCLE_MARGIN_PX = static_cast<int>(0.5 * PX_PER_M);

// T자 특징점 감지 시 추가 서클 마진 (1.5m to 픽셀)
constexpr int T_SHAPE_CIRCLE_MARGIN_PX = static_cast<int>(1.5 * PX_PER_M);

// mm → 맵 픽셀 변환 상수
constexpr double MM2PX = PX_PER_M / 1000.0;

// 필드 경계 픽셀 상수 (sel_ball, obstacle 클램핑에 사용)
constexpr int F_X_MIN = toPixelX(-A / 2); // 필드 왼쪽 경계
constexpr int F_X_MAX = toPixelX(A / 2);  // 필드 오른쪽 경계
constexpr int F_Y_MIN = toPixelY(B / 2);  // 필드 위쪽 경계
constexpr int F_Y_MAX = toPixelY(-B / 2); // 필드 아래쪽 경계 

// 공 신뢰 가장자리 마진 (0.65m → 픽셀).
// 필드 경계 근처에서 계산된 공은 오차가 커서 신뢰 박스를 안쪽으로 들여놓는다.
constexpr int BALL_EDGE_MARGIN_PX = static_cast<int>(0.65 * PX_PER_M);
constexpr int BALL_TRUST_X_MIN = F_X_MIN + BALL_EDGE_MARGIN_PX;
constexpr int BALL_TRUST_X_MAX = F_X_MAX - BALL_EDGE_MARGIN_PX;
constexpr int BALL_TRUST_Y_MIN = F_Y_MIN + BALL_EDGE_MARGIN_PX;
constexpr int BALL_TRUST_Y_MAX = F_Y_MAX - BALL_EDGE_MARGIN_PX;

// 특징점 정의
// ============================================================

enum FeatureType { L_SHAPE, T_SHAPE, X_SHAPE, PENALTY_MARK, GOAL_POST };

struct FeaturePoint
{
  double      x;      // 필드 X 좌표
  double      y;      // 필드 Y 좌표
  FeatureType type;
  bool        active = true;  // false: 시각화 전용, 가중치 계산 제외
};

//ㄱ-shape (L) - 12개
// 필드 외곽 코너 4개
constexpr FeaturePoint L00 = { -A/2,  B/2, L_SHAPE};  // 좌상단 코너
constexpr FeaturePoint L01 = {  A/2,  B/2, L_SHAPE};  // 우상단 코너
constexpr FeaturePoint L02 = { -A/2, -B/2, L_SHAPE};  // 좌하단 코너
constexpr FeaturePoint L03 = {  A/2, -B/2, L_SHAPE};  // 우하단 코너
// 좌측 페널티 에어리어 원거리 코너 2개
constexpr FeaturePoint L04 = { -A/2+G,  H/2, L_SHAPE };
constexpr FeaturePoint L05 = { -A/2+G, -H/2, L_SHAPE };
// 우측 페널티 에어리어 원거리 코너 2개
constexpr FeaturePoint L06 = {  A/2-G,  H/2, L_SHAPE };
constexpr FeaturePoint L07 = {  A/2-G, -H/2, L_SHAPE };
// 좌측 골 에어리어 원거리 코너 2개
constexpr FeaturePoint L08 = { -A/2+E,  F/2, L_SHAPE };
constexpr FeaturePoint L09 = { -A/2+E, -F/2, L_SHAPE };
// 우측 골 에어리어 원거리 코너 2개
constexpr FeaturePoint L10 = {  A/2-E,  F/2, L_SHAPE };
constexpr FeaturePoint L11 = {  A/2-E, -F/2, L_SHAPE };

//T-shape - 10개
// 하프라인 × 터치라인 2개
constexpr FeaturePoint T00 = {  0,  B/2, T_SHAPE };  // 상단 중앙
constexpr FeaturePoint T01 = {  0, -B/2, T_SHAPE };  // 하단 중앙
// 좌측 골라인: 페널티 에어리어 사이드라인 2개
constexpr FeaturePoint T02 = { -A/2,  H/2, T_SHAPE };
constexpr FeaturePoint T03 = { -A/2, -H/2, T_SHAPE };
// 좌측 골라인: 골 에어리어 사이드라인 2개
constexpr FeaturePoint T04 = { -A/2,  F/2, T_SHAPE };
constexpr FeaturePoint T05 = { -A/2, -F/2, T_SHAPE };
// 우측 골라인: 페널티 에어리어 사이드라인 2개
constexpr FeaturePoint T06 = {  A/2,  H/2, T_SHAPE };
constexpr FeaturePoint T07 = {  A/2, -H/2, T_SHAPE };
// 우측 골라인: 골 에어리어 사이드라인 2개
constexpr FeaturePoint T08 = {  A/2,  F/2, T_SHAPE };
constexpr FeaturePoint T09 = {  A/2, -F/2, T_SHAPE };

// X-shape - 2개

// 하프라인 × 센터 서클 상단 교차점
constexpr FeaturePoint X01 = {  0,  J/2, X_SHAPE };
// 하프라인 × 센터 서클 하단 교차점
constexpr FeaturePoint X02 = {  0, -J/2, X_SHAPE };

constexpr FeaturePoint  Left_Goalpost = { -A/2, 0, GOAL_POST};
constexpr FeaturePoint Right_Goalpost = {  A/2, 0, GOAL_POST};
// 전체 특징점 배열 - 26개
constexpr FeaturePoint ALL_POINTS[] = {
  // ㄱ-shape (index 0~11)
  L00, L01, L02, L03, L04, L05, L06, L07, L08, L09, L10, L11,
  // T-shape  (index 12~21)
  T00, T01, T02, T03, T04, T05, T06, T07, T08, T09,
  // X-shape  (index 22~23)
  X01, X02,
  // 골대 (index 24~25)
  Left_Goalpost, Right_Goalpost
};

constexpr int FEATURE_POINT_COUNT = sizeof(ALL_POINTS) / sizeof(ALL_POINTS[0]);

} // namespace FieldCoord
