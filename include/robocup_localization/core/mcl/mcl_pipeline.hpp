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
        double min_fit = 0.0;
        bool noise_enabled = true;
        int threads = 0;
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

        std::vector<VISION_POINT> vision_point_vect;
        int vision_data_count = 0;
        int vision_frame_count = 0;
        bool valid_observation = false;

        double ball_relative_x_mm = 0.0;
        double ball_relative_y_mm = 0.0;

        PreFilter filter;

        int robot_y_sign = 0;  // from goalposts: 1 = y > 0, -1 = y < 0, 0 = unknown

        std::vector<MEASUREMENT> particle_weight;  // sorted by weight, descending

        // Particles exactly as weighted in the last cycle, before resampling (for the UI).
        std::vector<Particle> scored_particles;
        std::vector<double> scored_weights;  // same order as scored_particles

        static constexpr int MIN_OBSERVATION_COUNT = 10;

        // Pose updates outside [MIN_JUMP, MAX_JUMP) are rejected: jitter below, no global jumps above.
        static constexpr double MIN_JUMP_M = MclTune::MIN_JUMP_M;
        static constexpr double MAX_JUMP_M = MclTune::MAX_JUMP_M;

        static constexpr double MAX_SPREAD_M = MclTune::MAX_SPREAD_M;

    private:
        void clearObservations();
        MeasurementContext buildContext(const MclInput &input);
        void appendSyntheticObservations(const MclInput &input, const MclResult &result);
    };

}
