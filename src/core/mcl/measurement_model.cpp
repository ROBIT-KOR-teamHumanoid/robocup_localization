#include "robocup_localization/core/mcl/measurement_model.hpp"

#include <algorithm>
#include <cmath>

namespace robocup_localization::core
{

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    return x.WEIGHT > y.WEIGHT;
}

PreparedObservations MeasurementModel::prepare(
    const std::vector<VISION_POINT>& observations,
    float yaw_deg,
    const PreFilter& filter,
    const MeasurementContext& context)
{
    PreparedObservations prepared;
    prepared.yaw_deg = yaw_deg;
    prepared.ball_visible = context.ball_visible;

    for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
    {
        prepared.landmark_x[j] = filter.Local_point_x[j];
        prepared.landmark_y[j] = filter.Local_point_y[j];
        prepared.exp_ball_angle[j] =
            std::atan2(filter.Local_point_y[j] - context.ball_field_px_y,
                       filter.Local_point_x[j] - context.ball_field_px_x);
    }

    const double yaw_rad = yaw_deg * DEG2RAD;
    const double cp = std::cos(yaw_rad);
    const double sp = std::sin(yaw_rad);

    prepared.observations.reserve(observations.size());
    for (const VISION_POINT& observation : observations)
    {
        PreparedObservation out;
        out.confidence = observation.CONFIDENCE;
        out.distance_m = observation.DISTANCE / 1000.0;
        out.l_shape =
            (FieldCoord::FeatureType)observation.FEATURE_TYPE == FieldCoord::L_SHAPE;

        const double raw_x_m = observation.RAW_X_MM / 1000.0;
        const double raw_y_m = observation.RAW_Y_MM / 1000.0;
        out.vec_x = (int)((-raw_x_m * cp - raw_y_m * sp) * FieldCoord::PX_PER_M_X);
        out.vec_y = (int)(( raw_x_m * sp - raw_y_m * cp) * FieldCoord::PX_PER_M_Y);
        out.obs_angle = std::atan2(out.vec_y, out.vec_x);

        const double obs_feat_field_x = VisionPointUtils::worldX(observation);
        const double obs_feat_field_y = VisionPointUtils::worldY(observation);
        out.obs_ball_angle =
            std::atan2(obs_feat_field_y - context.ball_field_px_y,
                       obs_feat_field_x - context.ball_field_px_x);

        // Kept in index order so ties resolve to the same landmark as a full scan.
        for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
        {
            if (!FieldCoord::ALL_POINTS[j].active)
                continue;
            if (FieldCoord::ALL_POINTS[j].type != (FieldCoord::FeatureType)observation.FEATURE_TYPE)
                continue;
            if (context.robot_y_sign != 0 &&
                observation.FEATURE_TYPE == (int)FieldCoord::L_SHAPE &&
                FieldCoord::ALL_POINTS[j].y * context.robot_y_sign < 0.0)
                continue;
            out.candidates[out.candidate_count++] = j;
        }

        prepared.observations.push_back(out);
    }
    return prepared;
}

double MeasurementModel::logLikelihood(
    int pt_x, int pt_y, const PreparedObservations& prepared)
{
    const int* lx = prepared.landmark_x;
    const int* ly = prepared.landmark_y;

    // Particle-to-landmark bearing and range are shared by all observations, so compute each once.
    double exp_angle[FieldCoord::FEATURE_POINT_COUNT];
    double actual_dist_m[FieldCoord::FEATURE_POINT_COUNT];
    bool have_angle[FieldCoord::FEATURE_POINT_COUNT] = {};
    bool have_dist[FieldCoord::FEATURE_POINT_COUNT] = {};

    double log_weight = 0.0;

    for (const PreparedObservation& observation : prepared.observations)
    {
        const int ptx = pt_x + observation.vec_x;
        const int pty = pt_y + observation.vec_y;

        double min_dis = LARGE_DIST;
        int    best_j  = -1;
        double best_ball_angle_diff = LARGE_DIST;

        for (int c = 0; c < observation.candidate_count; c++)
        {
            const int j = observation.candidates[c];

            if (!have_angle[j])
            {
                exp_angle[j] = std::atan2(ly[j] - pt_y, lx[j] - pt_x);
                have_angle[j] = true;
            }
            if (!PreFilter::passBearing(observation.obs_angle, exp_angle[j]))
                continue;

            const double dx_m = (lx[j] - ptx) / FieldCoord::PX_PER_M_X;
            const double dy_m = (ly[j] - pty) / FieldCoord::PX_PER_M_Y;
            const double dis  = std::sqrt(dx_m * dx_m + dy_m * dy_m);

            if (observation.l_shape)
            {
                if (!have_dist[j])
                {
                    actual_dist_m[j] =
                        std::sqrt(std::pow((lx[j] - pt_x) / FieldCoord::PX_PER_M_X, 2)
                                + std::pow((ly[j] - pt_y) / FieldCoord::PX_PER_M_Y, 2));
                    have_dist[j] = true;
                }
                if (std::fabs(actual_dist_m[j] - observation.distance_m) > L_DIST_GATE_M)
                    continue;
            }

            if (prepared.ball_visible)
            {
                double ball_angle_diff =
                    std::fabs(observation.obs_ball_angle - prepared.exp_ball_angle[j]);
                if (ball_angle_diff > M_PI)
                    ball_angle_diff = 2 * M_PI - ball_angle_diff;

                if (ball_angle_diff < best_ball_angle_diff)
                {
                    best_ball_angle_diff = ball_angle_diff;
                    min_dis = dis;
                    best_j  = j;
                }
            }
            else
            {
                if (dis < min_dis)
                {
                    min_dis = dis;
                    best_j  = j;
                }
            }
        }

        if (best_j < 0)
        {
            log_weight += observation.confidence * MIN_LOG_LIKELIHOOD;
            continue;
        }

        // Convert each axis with its own scale before decomposing: x/y px per meter differ
        // (45.8 / 50 on LARGE), so one shared factor skews the error by direction.
        const double err_x_m = (ptx - lx[best_j]) / FieldCoord::PX_PER_M_X;
        const double err_y_m = (pty - ly[best_j]) / FieldCoord::PX_PER_M_Y;

        const double vx_m   = (lx[best_j] - pt_x) / FieldCoord::PX_PER_M_X;
        const double vy_m   = (ly[best_j] - pt_y) / FieldCoord::PX_PER_M_Y;
        const double dist_m = std::max(1e-6, std::sqrt(vx_m * vx_m + vy_m * vy_m));
        const double ux     = vx_m / dist_m;
        const double uy     = vy_m / dist_m;

        const double dn_m = ux * err_x_m + uy * err_y_m;
        const double dp_m = -uy * err_x_m + ux * err_y_m;

        const double sigma_n = INV_NORM_VAR * dist_m + 0.05;
        const double sigma_p = INV_PERP_VAR * dist_m + 0.05;

        const double D2 =
            (dn_m * dn_m) / (sigma_n * sigma_n) + (dp_m * dp_m) / (sigma_p * sigma_p);
        log_weight += observation.confidence * std::max(-0.5 * D2, MIN_LOG_LIKELIHOOD);
    }

    return log_weight;
}

double MeasurementModel::logLikelihood(
    const Particle& particle,
    const std::vector<VISION_POINT>& observations,
    const PreFilter& filter,
    const MeasurementContext& context)
{
    // Particles are evaluated on the pixel grid (float -> int truncation is legacy behavior).
    const PreparedObservations prepared =
        prepare(observations, particle.yaw_deg, filter, context);
    return logLikelihood(
        static_cast<int>(particle.x), static_cast<int>(particle.y), prepared);
}

}
