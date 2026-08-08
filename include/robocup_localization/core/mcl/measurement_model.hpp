#pragma once

#include <vector>

#include "field_coordinate/field_coordinate.hpp"
#include "robocup_localization/core/mcl/mcl_tune.hpp"
#include "robocup_localization/core/mcl/observation_prefilter.hpp"
#include "robocup_localization/core/types/particle.hpp"
#include "robocup_localization/core/types/vision_point.hpp"

namespace robocup_localization::core
{

    struct MEASUREMENT
    {
        int NUM;
        double WEIGHT;
    };

    bool sort_return(MEASUREMENT x, MEASUREMENT y);

    struct MeasurementContext
    {
        double ball_field_px_x = 0.0;
        double ball_field_px_y = 0.0;
        bool ball_visible = false;
        int robot_y_sign = 0;
    };

    class MeasurementModel
    {
    public:
        /// 파티클 하나에 대한 누적 가중치
        static double particleWeight(
            const Particle &particle,
            const std::vector<VISION_POINT> &observations,
            const PreFilter &filter,
            const MeasurementContext &context);

        // min_dis 초기값. 가장 가까운 랜드마크를 찾기 전 비교 기준값
        static constexpr double LARGE_DIST = 99999999.0;

        // normal 방향 분산 계수 (거리 비례). 클수록 normal 오차에 관대해짐.
        static constexpr double INV_NORM_VAR = 0.1;

        // perp 방향 분산 계수. INV_NORM_VAR보다 크게 설정 → 수직 오차에 더 관대.
        static constexpr double INV_PERP_VAR = 0.2;

        // L_SHAPE 거리 매칭 게이트 (미터). 관측 거리와 실제 거리 차이가 이 값 초과이면 skip.
        static constexpr double L_DIST_GATE_M = MclTune::L_DIST_GATE_M;
    };

} // namespace robocup_localization::core
