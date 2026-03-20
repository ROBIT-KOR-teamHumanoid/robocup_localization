#pragma once

#include "robot_config.h"  // FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE 정의

// ============================================================
// field_coordinate.hpp
//
// RoboCup 필드 치수 및 특징점 좌표 정의
//
// 유지보수:
//   robot_config.h 에서 아래 중 하나만 활성화
//
//     2026 RoboCup 규정을 기준으로 함
//     #define FIELD_SMALL    // S-Field: 9m × 6m  
//     #define FIELD_MIDDLE   // M-Field: 14m × 9m
//     #define FIELD_LARGE    // L-Field: 22m × 14m 
// 좌표계:
//   필드 좌표 - 원점: 필드 중앙, x: 오른쪽(+), y: 위쪽(+), 단위: 미터
//   픽셀 좌표 - 원점: 이미지 좌상단, x: 오른쪽(+), y: 아래쪽(+)
//
// 특징점 총 27개:
//   ㄱ-shape (L) : 12개  (인덱스 0~11)
//   T-shape      : 10개  (인덱스 12~21)
//   X-shape      :  3개  (인덱스 22~24)
//   페널티 마크  :  2개  (인덱스 25~26)
// ============================================================

namespace FieldCoord {

// 필드 치수
// 범위가 있는 항목은 최댓값 사용

#if defined(FIELD_SMALL)
  // S-Field 
  constexpr double A      = 9.0;   // 필드 길이 (Field length)
  constexpr double B      = 6.0;   // 필드 너비 (Field width)
  constexpr double C      = 1.0;   // 골 깊이   (Goal depth)
  constexpr double D      = 1.9;   // 골 너비   (Goal width)
  constexpr double E      = 1.0;   // 골 에어리어 길이 (Goal area length)
  constexpr double F      = 3.0;   // 골 에어리어 너비 (Goal area width)
  constexpr double G      = 2.0;   // 페널티 에어리어 길이 (Penalty area length)
  constexpr double H      = 5.0;   // 페널티 에어리어 너비 (Penalty area width), 규정보다 1m 더 큼, 이미지가 이상해서 그럼, 이미지를 다시 그려야 하나? 얘만 이상하고 다른 값들은 규정대로 해도 잘 되던데
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
  constexpr double LINE_W = 0.05;
  constexpr double MARK_SZ= 0.10;

#elif defined(FIELD_LARGE)
  // L-Field
  constexpr double A      = 22.0;
  constexpr double B      = 14.0;
  constexpr double C      = 2.0;  
  constexpr double D      = 3.1;  
  constexpr double E      = 1.0;
  constexpr double F      = 5.0;
  constexpr double G      = 3.5;
  constexpr double H      = 7.0;
  constexpr double I_DIST = 2.5;
  constexpr double J      = 4.0;
  constexpr double K      = 2.4; // 아무나 수정좀해주라  이거는 필드 크기에 따라 보정값이 달라질 수 있음. 실제로는 1.0m인데 맵 뷰에서 1.0m보다 더 크게 보이는 경우 보정값을 1.0보다 크게 설정하여 시각적으로 1.0m에 맞게 보이도록 조정
  constexpr double L_ARC  = 1.0;
  constexpr double LINE_W = 0.12;
  constexpr double MARK_SZ= 0.15;

#else
  #error "필드 타입 미정의. robot_config.h 에 FIELD_SMALL / FIELD_MIDDLE / FIELD_LARGE 중 하나를 정의하세요."
#endif

// 맵 ui 파라미터
// ============================================================
constexpr int    MAP_W    = 800;           // 맵 이미지 너비 
constexpr int    MAP_H    = 600;           // 맵 이미지 높이 

constexpr double WORLD_W  = A + 2.0 * K;  // 표시 총 너비 (미터) = 필드 + 양쪽 보더
constexpr double WORLD_H  = B + 2.0 * K;  // 표시 총 높이 (미터)

constexpr double PX_PER_M_X = MAP_W / WORLD_W;  // X축 스케일 
constexpr double PX_PER_M_Y = MAP_H / WORLD_H;  // Y축 스케일 


// 좌표 변환 함수
// ============================================================

/// 필드 미터 좌표 -> 이미지 픽셀 X
constexpr int toPixelX(double fx)
{
  return static_cast<int>((fx + A * 0.5 + K) * PX_PER_M_X);
}

/// 필드 미터 좌표 -> 이미지 픽셀 Y  (Y축 반전: 필드 위쪽 = 픽셀 위쪽)
constexpr int toPixelY(double fy)
{
  return static_cast<int>((B * 0.5 + K - fy) * PX_PER_M_Y);
}

/// 이미지 픽셀 X -> 필드 미터 좌표
constexpr double toFieldX(double px)
{
  return px / PX_PER_M_X - A * 0.5 - K;
}

/// 이미지 픽셀 Y -> 필드 미터 좌표  (Y축 반전)
constexpr double toFieldY(double py)
{
  return B * 0.5 + K - py / PX_PER_M_Y;
}

// 파티클 필터 서클 마진 (0.5m → 픽셀)
constexpr int CIRCLE_MARGIN_PX = static_cast<int>(0.5 * PX_PER_M_X);

// mm → 맵 픽셀 변환 상수
constexpr double MM2PX = PX_PER_M_X / 1000.0;

// 필드 경계 픽셀 상수 (sel_ball, obstacle 클램핑에 사용)
constexpr int F_X_MIN = toPixelX(-A / 2); // 필드 왼쪽 경계
constexpr int F_X_MAX = toPixelX(A / 2);  // 필드 오른쪽 경계
constexpr int F_Y_MIN = toPixelY(B / 2);  // 필드 위쪽 경계 (작은 px)
constexpr int F_Y_MAX = toPixelY(-B / 2); // 필드 아래쪽 경계 (큰 px)

// 9-zone 경계 픽셀 상수
constexpr int COL_L  = toPixelX(-I_DIST);  // 좌측 컬럼 경계
constexpr int COL_R  = toPixelX( I_DIST);  // 우측 컬럼 경계
constexpr int ROW_T  = toPixelY( E);       // 외곽 컬럼 상단 행 경계
constexpr int ROW_B  = toPixelY(-E);       // 외곽 컬럼 하단 행 경계
constexpr int ROW_TC = toPixelY( H * 0.5); // 중앙 컬럼 상단 행 경계
constexpr int ROW_BC = toPixelY(-H * 0.5); // 중앙 컬럼 하단 행 경계

// 특징점 정의
// ============================================================

enum FeatureType { L_SHAPE, T_SHAPE, X_SHAPE, PENALTY_MARK };

struct FeaturePoint
{
  double      x;     // 필드 X 좌표 
  double      y;     // 필드 Y 좌표 
  FeatureType type;
};

//ㄱ-shape (L) - 12개
// 필드 외곽 코너 4개
constexpr FeaturePoint L00 = { -A/2,  B/2, L_SHAPE };  // 좌상단 코너
constexpr FeaturePoint L01 = {  A/2,  B/2, L_SHAPE };  // 우상단 코너
constexpr FeaturePoint L02 = { -A/2, -B/2, L_SHAPE };  // 좌하단 코너
constexpr FeaturePoint L03 = {  A/2, -B/2, L_SHAPE };  // 우하단 코너
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

// X-shape - 3개 
// 필드 중앙 (하프라인 × 센터 마크)
constexpr FeaturePoint X00 = {  0,     0, X_SHAPE };
// 하프라인 × 센터 서클 상단 교차점
constexpr FeaturePoint X01 = {  0,  J/2, X_SHAPE };
// 하프라인 × 센터 서클 하단 교차점
constexpr FeaturePoint X02 = {  0, -J/2, X_SHAPE };

//패널티 마크 - 2개
constexpr FeaturePoint PM0 = { -A/2 + I_DIST, 0, PENALTY_MARK };  // 좌측
constexpr FeaturePoint PM1 = {  A/2 - I_DIST, 0, PENALTY_MARK };  // 우측

// 전체 특징점 배열 - 27개 
constexpr FeaturePoint ALL_POINTS[27] = {
  // ㄱ-shape (index 0~11)
  L00, L01, L02, L03, L04, L05, L06, L07, L08, L09, L10, L11,
  // T-shape  (index 12~21)
  T00, T01, T02, T03, T04, T05, T06, T07, T08, T09,
  // X-shape  (index 22~24)
  X00, X01, X02,
  // 페널티 마크 (index 25~26)
  PM0, PM1
};

constexpr int FEATURE_POINT_COUNT = 27;

// ============================================================
// 존별 랜드마크 인덱스 테이블 (compile-time 자동 생성)
//
// 필드 크기(FIELD_SMALL/MIDDLE/LARGE)가 바뀌어도 자동으로 재계산됨
// ============================================================

/// ALL_POINTS[i]가 속하는 존 번호 반환 (1~9, 범위 밖은 0)
constexpr int getZone(const FeaturePoint& p)
{
    if (p.x < -I_DIST) {
        // 좌측 열
        if (p.y >  E)  return 1;
        if (p.y > -E)  return 4;
        return 7;
    } else if (p.x <= I_DIST) {
        // 중앙 열
        if (p.y >  H * 0.5)  return 2;
        if (p.y > -H * 0.5)  return 5;
        return 8;
    } else {
        // 우측 열
        if (p.y >  E)  return 3;
        if (p.y > -E)  return 6;
        return 9;
    }
}

struct ZoneLandmarks
{
    int indices[FEATURE_POINT_COUNT];
    int count;
};

constexpr ZoneLandmarks computeZoneLandmarks(int zone)
{
    ZoneLandmarks result{};
    result.count = 0;
    for (int i = 0; i < FEATURE_POINT_COUNT; i++)
        if (getZone(ALL_POINTS[i]) == zone)
            result.indices[result.count++] = i;
    return result;
}

/// 존 번호(1~9)별 랜드마크 인덱스 목록. ZONE_LANDMARKS[0]은 미사용.
constexpr ZoneLandmarks ZONE_LANDMARKS[10] = {
    {},                       // zone 0 (범위 밖)
    computeZoneLandmarks(1),
    computeZoneLandmarks(2),
    computeZoneLandmarks(3),
    computeZoneLandmarks(4),
    computeZoneLandmarks(5),
    computeZoneLandmarks(6),
    computeZoneLandmarks(7),
    computeZoneLandmarks(8),
    computeZoneLandmarks(9),
};

} // namespace FieldCoord
