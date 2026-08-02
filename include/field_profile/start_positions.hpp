#pragma once

#include "field_profile/field_coordinate.hpp"

// 경기 시작·복귀 위치 프리셋
// ============================================================
// 진영과 역할에 따라 정해진 자리들. 값은 규칙이지 로직이 아니므로 표로 둔다.
//
// 표 보는 법 — 한 줄이 한 자리다.
//
//     { {자기진영 x, 터치라인}, {상대진영 x, 터치라인} }
//
// y 는 항상 터치라인(±B/2) 위라 Top/Bottom 으로만 적는다. 필드 크기가
// 런타임에 정해지므로 실제 픽셀 좌표는 조회 시점에 계산된다.
//
// 진영이 바뀔 때 단순 미러가 아닌 점에 주의 (0.8 ↔ -3.5). 슬롯 번호가
// 필드상의 위치가 아니라 **역할**을 가리키기 때문이다.
namespace StartPositions
{

// y = +B/2 (위쪽 터치라인) / -B/2 (아래쪽)
enum class Touchline
{
  Top,
  Bottom
};

struct StartPose
{
  double x_m;
  Touchline touchline;
};

// 같은 역할의 두 진영 값. gameMSG.mySide 로 고른다.
struct SideVariants
{
  StartPose own_half;  // mySide != 0
  StartPose opp_half;  // mySide == 0
};

// ── 수동 배치 (UI 버튼 1~6) ────────────────────────────────────
constexpr SideVariants MANUAL[6] = {
    /* 1 */ {{0.8, Touchline::Top}, {-3.5, Touchline::Top}},
    /* 2 */ {{2.5, Touchline::Top}, {-2.5, Touchline::Top}},
    /* 3 */ {{3.5, Touchline::Top}, {-0.8, Touchline::Top}},
    /* 4 */ {{0.8, Touchline::Bottom}, {-3.5, Touchline::Bottom}},
    /* 5 */ {{2.5, Touchline::Bottom}, {-2.5, Touchline::Bottom}},
    /* 6 */ {{3.5, Touchline::Bottom}, {-0.8, Touchline::Bottom}},
};

// ── get in 자동 배치 (robotNum 1~4) ────────────────────────────
constexpr SideVariants AUTO[4] = {
    /* 1번 로봇 */ {{3.5, Touchline::Bottom}, {-3.5, Touchline::Top}},
    /* 2번 로봇 */ {{0.8, Touchline::Bottom}, {-0.8, Touchline::Top}},
    /* 3번 로봇 */ {{2.5, Touchline::Top}, {-2.5, Touchline::Bottom}},
    /* 4번 로봇 */ {{0.8, Touchline::Top}, {-0.8, Touchline::Bottom}},
};

// ── 반칙 후 복귀 ───────────────────────────────────────────────
constexpr SideVariants PENALTY[2] = {
    /* position == 1 */ {{3.5, Touchline::Bottom}, {-3.5, Touchline::Bottom}},
    /* 그 외          */ {{2.5, Touchline::Bottom}, {-2.5, Touchline::Bottom}},
};

// ── 조회 ───────────────────────────────────────────────────────

struct PosePx
{
  float x = 0.0f;
  float y = 0.0f;
};

inline PosePx toPx(const StartPose &p)
{
  const double y_m =
      (p.touchline == Touchline::Top) ? FieldCoord::B / 2 : -FieldCoord::B / 2;
  return PosePx{static_cast<float>(FieldCoord::toPixelX(p.x_m)),
                static_cast<float>(FieldCoord::toPixelY(y_m))};
}

inline PosePx pick(const SideVariants &v, bool my_side)
{
  return toPx(my_side ? v.own_half : v.opp_half);
}

/// 수동 배치 버튼. slot 은 1~6, 벗어나면 1번으로 떨어진다.
inline PosePx manualSlot(int slot, bool my_side)
{
  const int i = (slot >= 1 && slot <= 6) ? slot - 1 : 0;
  return pick(MANUAL[i], my_side);
}

/// get in 자동 배치. robot_num 은 1~4, 벗어나면 (0,0).
inline PosePx autoByRobotNum(int robot_num, bool my_side)
{
  if (robot_num < 1 || robot_num > 4)
    return PosePx{};
  return pick(AUTO[robot_num - 1], my_side);
}

/// 반칙 후 복귀 위치. position 이 1 이면 첫 번째 자리, 아니면 두 번째.
inline PosePx penaltyReturn(int position, bool my_side)
{
  return pick(PENALTY[(position == 1) ? 0 : 1], my_side);
}

}  // namespace StartPositions
