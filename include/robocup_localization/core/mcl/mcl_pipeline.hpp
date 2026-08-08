#pragma once

#include <vector>

#include "field_coordinate/field_coordinate.hpp"
#include "robocup_localization/core/mcl/mcl_tune.hpp"
#include "robocup_localization/core/mcl/measurement_model.hpp"
#include "robocup_localization/core/mcl/observation_prefilter.hpp"
#include "robocup_localization/core/mcl/resampler.hpp"
#include "robocup_localization/core/types/particle.hpp"
#include "robocup_localization/core/types/vision_point.hpp"

namespace robocup_localization::core
{

    struct MclInput
    {
        float robot_x = 0.0f;
        float robot_y = 0.0f;
        float robot_yaw_deg = 0.0f;
        double particle_range = 0.0;
        double weight_update_threshold = 0.0;
        bool noise_enabled = true;
    };

    struct MclResult
    {
        bool pose_updated = false;
        float robot_x = 0.0f;
        float robot_y = 0.0f;
    };

    class MclPipeline
    {
    public:
        MclResult run(std::vector<Particle> &particles, const MclInput &input);

        // ---- 관측 버퍼 ----
        std::vector<VISION_POINT> vision_point_vect;
        int vision_data_count = 0;
        int vision_data_size = 0;
        bool valid_observation = false;

        // ---- 공 관측 ----
        double ball_relative_x_mm = 0.0;
        double ball_relative_y_mm = 0.0;

        PreFilter filter;

        // 골대 관측으로 결정된 로봇 y부호 (1: y>0, -1: y<0, 0: 미결정)
        int robot_y_sign = 0;

        // 마지막 사이클의 파티클별 가중치 (가중치 내림차순 정렬됨)
        std::vector<MEASUREMENT> particle_weight;

        // ---- 포즈 채택 게이트 ----
        // 관측이 이 개수 이하이면 스킵
        static constexpr int MIN_OBSERVATION_COUNT = 10;

        // 한 번의 매칭에서 로봇 위치가 이동할 수 있는 최소/최대 거리 (미터).
        // MIN_JUMP 미만이면 진동 억제를 위해 업데이트 skip.
        static constexpr double MIN_JUMP_M = MclTune::MIN_JUMP_M;
        static constexpr double MIN_JUMP_PX = MIN_JUMP_M * FieldCoord::PX_PER_M_X;
        static constexpr double MAX_JUMP_M = MclTune::MAX_JUMP_M;
        static constexpr double MAX_JUMP_PX = MAX_JUMP_M * FieldCoord::PX_PER_M_X;

        // top-N 파티클 위치 분산 허용 한계. 이보다 퍼져있으면 업데이트 reject.
        static constexpr double MAX_SPREAD_M = MclTune::MAX_SPREAD_M;
        static constexpr double MAX_SPREAD_PX = MAX_SPREAD_M * FieldCoord::PX_PER_M_X;

    private:
        void clearObservations();
        MeasurementContext buildContext(const MclInput &input);
        void appendSyntheticObservations(const MclInput &input, const MclResult &result);
    };

} // namespace robocup_localization::core
