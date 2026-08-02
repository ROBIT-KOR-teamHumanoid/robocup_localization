#pragma once

#include <string>

#include "field_profile/field_coordinate.hpp"  // PX_PER_M (픽셀 환산)

// MCL 필터 튜닝값 (미터)
// ============================================================
// 필드 크기에 따라 값이 달라지는 거리 도메인 임계값들.
// 좌표계(FieldCoord)가 아니라 필터의 관심사라 규격(field_spec.hpp)과 분리한다.
// 각도·무차원 계수(BEARING_MAX_RAD, INV_*_VAR 등)는 스케일 불변이라 제외.
//
// 예전엔 S 값에 FIELD_SCALE(=A/9) 을 곱했는데, 그러면 한 필드를 실측으로
// 조정할 때 나머지 필드까지 같이 움직였다. 그래서 필드별 실수값을 직접 적는다.
//
// 값의 출처
//   S : 실측 튜닝치 (2026-04~06, S-Field 에서 확정)
//   M : S x 1.556 (=14/9) — 미검증, 실측하면 이 자리를 직접 고칠 것
//   L : S x 2.444 (=22/9) — 미검증, 상동
//
//   MclTune::init(*MclTune::fromName("L"));   // 파라미터 읽은 직후 1회
namespace MclTune
{

    struct Small_Tune
    {
        static constexpr double OBS_FILTER_DIST_M = 1.7;           // pre-filter 거리 임계
        static constexpr double OBS_FILTER_X_FEATURE_DIST_M = 2.0; // X-shape 전용
        static constexpr double L_DIST_GATE_M = 0.5;               // L-shape 거리 매칭 게이트
        static constexpr double MIN_JUMP_M = 0.07;                 // 위치 보정 최소 (진동 억제)
        static constexpr double MAX_JUMP_M = 1.0;                  // 위치 보정 최대
        static constexpr double MAX_SPREAD_M = 0.5;                // top-N 파티클 분산 허용
    };

    struct Middle_Tune
    {
        static constexpr double OBS_FILTER_DIST_M = 2.64;
        static constexpr double OBS_FILTER_X_FEATURE_DIST_M = 3.11;
        static constexpr double L_DIST_GATE_M = 0.78;
        static constexpr double MIN_JUMP_M = 0.11;
        static constexpr double MAX_JUMP_M = 1.56;
        static constexpr double MAX_SPREAD_M = 0.78;
    };

    struct Large_Tune
    {
        static constexpr double OBS_FILTER_DIST_M = 4.16;
        static constexpr double OBS_FILTER_X_FEATURE_DIST_M = 4.89;
        static constexpr double L_DIST_GATE_M = 1.22;
        static constexpr double MIN_JUMP_M = 0.17;
        static constexpr double MAX_JUMP_M = 2.44;
        static constexpr double MAX_SPREAD_M = 1.22;
    };

    // 위 튜닝값들을 담는 런타임 값 묶음.
    struct TuneSpec
    {
        double OBS_FILTER_DIST_M;
        double OBS_FILTER_X_FEATURE_DIST_M;
        double L_DIST_GATE_M;
        double MIN_JUMP_M;
        double MAX_JUMP_M;
        double MAX_SPREAD_M;
    };

    template <class Tune>
    constexpr TuneSpec makeTune()
    {
        return TuneSpec{Tune::OBS_FILTER_DIST_M,
                        Tune::OBS_FILTER_X_FEATURE_DIST_M,
                        Tune::L_DIST_GATE_M,
                        Tune::MIN_JUMP_M,
                        Tune::MAX_JUMP_M,
                        Tune::MAX_SPREAD_M};
    }

    inline constexpr TuneSpec SMALL = makeTune<Small_Tune>();
    inline constexpr TuneSpec MIDDLE = makeTune<Middle_Tune>();
    inline constexpr TuneSpec LARGE = makeTune<Large_Tune>();

    inline const TuneSpec *fromName(const std::string &name)
    {
        if (name == "S" || name == "SMALL")
            return &SMALL;
        if (name == "M" || name == "MIDDLE")
            return &MIDDLE;
        if (name == "L" || name == "LARGE")
            return &LARGE;
        return nullptr;
    }

    // 적용된 값. init() 전에는 전부 0 → 모든 게이트가 즉시 닫혀 매칭이
    // 전부 실패하므로, 의심되면 initialized() 로 확인할 것.
    inline double OBS_FILTER_DIST_M = 0.0;           // pre-filter 거리 임계
    inline double OBS_FILTER_X_FEATURE_DIST_M = 0.0; // X-shape 전용 pre-filter 거리 임계
    inline double L_DIST_GATE_M = 0.0;               // L-shape 거리 매칭 게이트
    inline double MIN_JUMP_M = 0.0;                  // 위치 보정 최소 (진동 억제)
    inline double MAX_JUMP_M = 0.0;                  // 위치 보정 최대
    inline double MAX_SPREAD_M = 0.0;                // top-N 파티클 분산 허용

    // 픽셀 환산은 FieldCoord::PX_PER_M 에 의존하므로 상수가 아니라 함수로 둔다.
    inline double minJumpPx() { return MIN_JUMP_M * FieldCoord::PX_PER_M; }
    inline double maxJumpPx() { return MAX_JUMP_M * FieldCoord::PX_PER_M; }
    inline double maxSpreadPx() { return MAX_SPREAD_M * FieldCoord::PX_PER_M; }

    namespace detail
    {
        inline bool g_initialized = false;
    }

    inline bool initialized() { return detail::g_initialized; }

    inline void init(const TuneSpec &tune)
    {
        OBS_FILTER_DIST_M = tune.OBS_FILTER_DIST_M;
        OBS_FILTER_X_FEATURE_DIST_M = tune.OBS_FILTER_X_FEATURE_DIST_M;
        L_DIST_GATE_M = tune.L_DIST_GATE_M;
        MIN_JUMP_M = tune.MIN_JUMP_M;
        MAX_JUMP_M = tune.MAX_JUMP_M;
        MAX_SPREAD_M = tune.MAX_SPREAD_M;
        detail::g_initialized = true;
    }

} // namespace MclTune