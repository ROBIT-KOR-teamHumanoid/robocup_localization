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

    // Per-observation values that depend only on the heading, not on the particle position.
    struct PreparedObservation
    {
        double confidence = 0.0;
        double distance_m = 0.0;
        bool l_shape = false;
        int vec_x = 0;
        int vec_y = 0;
        double obs_angle = 0.0;
        double obs_ball_angle = 0.0;
        int candidates[FieldCoord::FEATURE_POINT_COUNT] = {};
        int candidate_count = 0;
    };

    struct PreparedObservations
    {
        float yaw_deg = 0.0f;
        bool ball_visible = false;
        int landmark_x[FieldCoord::FEATURE_POINT_COUNT] = {};
        int landmark_y[FieldCoord::FEATURE_POINT_COUNT] = {};
        double exp_ball_angle[FieldCoord::FEATURE_POINT_COUNT] = {};
        std::vector<PreparedObservation> observations;
    };

    class MeasurementModel
    {
    public:
        // Confidence-weighted sum of per-observation log-likelihoods (a product of likelihoods).
        static double logLikelihood(
            const Particle &particle,
            const std::vector<VISION_POINT> &observations,
            const PreFilter &filter,
            const MeasurementContext &context);

        // Same result as above, split so the heading-only work runs once for many particles.
        static PreparedObservations prepare(
            const std::vector<VISION_POINT> &observations,
            float yaw_deg,
            const PreFilter &filter,
            const MeasurementContext &context);

        static double logLikelihood(int pt_x, int pt_y, const PreparedObservations &prepared);

        static constexpr double LARGE_DIST = 99999999.0;

        // Sigma grows with distance; perpendicular error is tolerated more than radial error.
        static constexpr double INV_NORM_VAR = 0.1;

        static constexpr double INV_PERP_VAR = 0.2;

        static constexpr double L_DIST_GATE_M = MclTune::L_DIST_GATE_M;

        // Floor at 3 sigma (also for unmatched observations) so one bad observation cannot zero a particle.
        static constexpr double MIN_LOG_LIKELIHOOD = -4.5;
    };

}
