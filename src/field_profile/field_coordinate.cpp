#include "field_profile/field_coordinate.hpp"

#include <algorithm>

namespace FieldCoord
{

// init() 전에는 전부 0. 그 상태로 읽으면 랜드마크가 (0,0) 에 몰려
// 매칭이 조용히 전부 실패한다 — initialized() 로 확인할 것.
FieldSpecs::FieldSpec field_profile_params = {};
FieldDerived field_profile_derived = {};

namespace
{
bool g_initialized = false;
}

bool initialized() { return g_initialized; }

void init(const FieldSpecs::FieldSpec &spec)
{
  field_profile_params = spec;

  // 파생값은 지역 d 에 전부 채운 뒤 마지막에 한 번 대입한다.
  // 전역을 순서대로 채우면서 toPixelX() 를 부르면, 아직 안 채워진 전역을
  // 읽게 되므로 계산 순서를 하나만 틀려도 조용히 어긋난다.
  FieldDerived d = {};

  // ── 1. 표시 스케일 (레터박스: 월드 영역을 맵 중앙에 배치) ────
  d.world_w = spec.A + 2.0 * spec.K;
  d.world_h = spec.B + 2.0 * spec.K;

  const double fit_x = MAP_W / d.world_w;
  const double fit_y = MAP_H / d.world_h;
  d.px_per_m = std::min(fit_x, fit_y);

  d.origin_x_px = (MAP_W - d.world_w * d.px_per_m) * 0.5;
  d.origin_y_px = (MAP_H - d.world_h * d.px_per_m) * 0.5;

  // 레터박스가 중앙 정렬이라 항상 MAP_H 와 같지만, 의도를 명시하려고 유도식으로 둔다.
  d.mirror_y_px = 2.0 * d.origin_y_px + d.world_h * d.px_per_m;

  d.mm2px = d.px_per_m / 1000.0;

  // 전역이 아직 옛 값이므로 헤더의 toPixelX/Y 대신 d 기준으로 변환한다.
  auto toPxX = [&](double fx) {
    return roundPx(d.origin_x_px + (fx + spec.A * 0.5 + spec.K) * d.px_per_m);
  };
  auto toPxY = [&](double fy) {
    return roundPx(d.origin_y_px + (spec.B * 0.5 + spec.K - fy) * d.px_per_m);
  };

  // ── 2. 마진·경계 ─────────────────────────────────────────────
  d.circle_margin_px = static_cast<int>(1.5 * d.px_per_m);
  d.t_shape_circle_margin_px = static_cast<int>(1.5 * d.px_per_m);

  d.f_x_min = toPxX(-spec.A / 2);
  d.f_x_max = toPxX(spec.A / 2);
  d.f_y_min = toPxY(spec.B / 2);
  d.f_y_max = toPxY(-spec.B / 2);

  d.ball_edge_margin_px = static_cast<int>(0.65 * d.px_per_m);
  d.ball_trust_x_min = d.f_x_min + d.ball_edge_margin_px;
  d.ball_trust_x_max = d.f_x_max - d.ball_edge_margin_px;
  d.ball_trust_y_min = d.f_y_min + d.ball_edge_margin_px;
  d.ball_trust_y_max = d.f_y_max - d.ball_edge_margin_px;

  // ── 3. 랜드마크 ──────────────────────────────────────────────
  // 골대는 active=false — MCL 가중치·prefilter·synthetic 에서 제외된다.
  // 관측(FEATURE_TYPE)은 L/T/X 세 종류뿐이라 타입 불일치로 어차피 스킵됐지만,
  // 그건 우연이라 의도를 플래그로 명시한다. 좌표 자체는 yaw_compensation 과
  // computeYSign 이 방향 기준으로 쓰므로 배열에 그대로 남긴다.
  const double a2 = spec.A / 2, b2 = spec.B / 2;
  const double f2 = spec.F / 2, h2 = spec.H / 2, j2 = spec.J / 2;

  const FeaturePoint pts[FEATURE_POINT_COUNT] = {
      // ㄱ-shape (index 0~11)
      {-a2, b2, L_SHAPE},   // 좌상단 코너
      {a2, b2, L_SHAPE},    // 우상단 코너
      {-a2, -b2, L_SHAPE},  // 좌하단 코너
      {a2, -b2, L_SHAPE},   // 우하단 코너
      {-a2 + spec.G, h2, L_SHAPE},   // 좌측 페널티 에어리어 원거리 코너
      {-a2 + spec.G, -h2, L_SHAPE},
      {a2 - spec.G, h2, L_SHAPE},    // 우측 페널티 에어리어 원거리 코너
      {a2 - spec.G, -h2, L_SHAPE},
      {-a2 + spec.E, f2, L_SHAPE},   // 좌측 골 에어리어 원거리 코너
      {-a2 + spec.E, -f2, L_SHAPE},
      {a2 - spec.E, f2, L_SHAPE},    // 우측 골 에어리어 원거리 코너
      {a2 - spec.E, -f2, L_SHAPE},

      // T-shape (index 12~21)
      {0, b2, T_SHAPE},    // 하프라인 x 터치라인 상단
      {0, -b2, T_SHAPE},   // 하프라인 x 터치라인 하단
      {-a2, h2, T_SHAPE},  // 좌측 골라인: 페널티 에어리어 사이드라인
      {-a2, -h2, T_SHAPE},
      {-a2, f2, T_SHAPE},  // 좌측 골라인: 골 에어리어 사이드라인
      {-a2, -f2, T_SHAPE},
      {a2, h2, T_SHAPE},   // 우측 골라인: 페널티 에어리어 사이드라인
      {a2, -h2, T_SHAPE},
      {a2, f2, T_SHAPE},   // 우측 골라인: 골 에어리어 사이드라인
      {a2, -f2, T_SHAPE},

      // X-shape (index 22~23) — 하프라인 x 센터 서클 교차점
      {0, j2, X_SHAPE},
      {0, -j2, X_SHAPE},

      // 골대 (index 24~25) — IDX_GOALPOST_L / _R 과 순서가 일치해야 한다
      {-a2, 0, GOAL_POST, false},
      {a2, 0, GOAL_POST, false},
  };

  for (int i = 0; i < FEATURE_POINT_COUNT; ++i)
    d.all_points[i] = pts[i];

  field_profile_derived = d;
  g_initialized = true;
}

}  // namespace FieldCoord
