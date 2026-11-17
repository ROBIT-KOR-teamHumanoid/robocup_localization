#include "robocup_localization/core/mcl/mcl_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "robocup_localization/core/mcl/random_util.hpp"

namespace robocup_localization::core
{

void MclPipeline::clearObservations()
{
    vision_point_vect.clear();
    particle_weight.clear();
    vision_data_count = 0;
    vision_frame_count = 0;
}

MeasurementContext MclPipeline::buildContext(const MclInput& input)
{
    MeasurementContext context;
    context.robot_y_sign = robot_y_sign;
    context.ball_visible =
        ball_relative_x_mm != 0.0 || ball_relative_y_mm != 0.0;

    const double yaw_rad = input.robot_yaw_deg * M_PI / 180.0;
    context.ball_field_px_x = input.robot_x + (-1) *
        (ball_relative_x_mm / 1000.0 * FieldCoord::PX_PER_M_X * std::cos(yaw_rad) +
         ball_relative_y_mm / 1000.0 * FieldCoord::PX_PER_M_X * std::sin(yaw_rad));
    context.ball_field_px_y = input.robot_y +
        (ball_relative_x_mm / 1000.0 * FieldCoord::PX_PER_M_Y * std::sin(yaw_rad) -
         ball_relative_y_mm / 1000.0 * FieldCoord::PX_PER_M_Y * std::cos(yaw_rad));

    return context;
}

// Visible-but-undetected landmarks become synthetic observations for the next cycle,
// which keeps particles from scattering when observations are sparse.
void MclPipeline::appendSyntheticObservations(
    const MclInput& input, const MclResult& result)
{
    int synthetic_cnt =
        static_cast<int>(vision_point_vect.size()) - vision_data_count;
    if (synthetic_cnt < 0)
    {
        synthetic_cnt = 0;
    }

    bool observed_types[3] = {false, false, false};
    for (int k = synthetic_cnt;
         k < static_cast<int>(vision_point_vect.size()); ++k)
    {
        const int type = vision_point_vect[k].FEATURE_TYPE;
        if (type >= 0 && type < 3)
        {
            observed_types[type] = true;
        }
    }

    filter.checkLocalPoint(synthetic_cnt, input.robot_yaw_deg, vision_point_vect);

    particle_weight.clear();
    vision_point_vect.clear();
    vision_data_count = 0;
    vision_frame_count = 0;

    for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; ++i)
    {
        if (!FieldCoord::ALL_POINTS[i].active ||
            filter.Local_point_check[i] != 1)
        {
            continue;
        }

        const int feature_type =
            static_cast<int>(FieldCoord::ALL_POINTS[i].type);
        if (feature_type < 0 || feature_type >= 3 ||
            !observed_types[feature_type])
        {
            continue;
        }
        if (robot_y_sign != 0 &&
            feature_type == static_cast<int>(FieldCoord::L_SHAPE) &&
            FieldCoord::ALL_POINTS[i].y * robot_y_sign < 0.0)
        {
            continue;
        }

        vision_point_vect.push_back(VisionPointUtils::makeSynthetic(
            filter.Local_point_x[i], filter.Local_point_y[i],
            result.robot_x, result.robot_y, feature_type,
            input.robot_yaw_deg));
    }
}

MclResult MclPipeline::run(
    std::vector<Particle>& particles, const MclInput& input)
{
    MclResult result;
    result.robot_x = input.robot_x;
    result.robot_y = input.robot_y;

    if (!valid_observation)
    {
        clearObservations();
        return result;
    }
    valid_observation = false;

    const int particle_count = static_cast<int>(particles.size());
    if (particle_count == 0)
    {
        clearObservations();
        return result;
    }

    const int obs_size = static_cast<int>(vision_point_vect.size());
    if (obs_size <= MIN_OBSERVATION_COUNT)
    {
        return result;
    }

    std::printf("PARTICLE UPDATE!!\n");

    const MeasurementContext context = buildContext(input);

    // Sparse matching: respawn particles around robot0.
    const bool small_matching =
        vision_frame_count > 0 &&
        vision_data_count <= vision_frame_count;

    if (small_matching)
    {
        for (auto& particle : particles)
        {
            randomizeParticle(
                particle, input.robot_x, input.robot_y,
                input.particle_range, input.robot_yaw_deg);
        }
    }

    filter.setCircle(input.robot_yaw_deg, vision_point_vect);
    filter.dropUnmatched(vision_point_vect);

    // Every particle carries the IMU yaw, so the heading-only work is shared by all of them.
    const PreparedObservations prepared = MeasurementModel::prepare(
        vision_point_vect, particles[0].yaw_deg, filter, context);

    std::vector<double> log_weights(particle_count);
    // Set per loop: omp_set_num_threads() would only affect the thread that called it.
    int threads = input.threads;
#ifdef _OPENMP
    if (threads <= 0)
    {
        threads = omp_get_max_threads();
    }
#endif
#pragma omp parallel for schedule(static) num_threads(threads)
    for (int i = 0; i < particle_count; ++i)
    {
        if (particles[i].yaw_deg == prepared.yaw_deg)
        {
            log_weights[i] = MeasurementModel::logLikelihood(
                static_cast<int>(particles[i].x), static_cast<int>(particles[i].y), prepared);
        }
        else
        {
            log_weights[i] = MeasurementModel::logLikelihood(
                particles[i], vision_point_vect, filter, context);
        }
    }

    double max_log_weight = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < particle_count; ++i)
    {
        max_log_weight = std::max(max_log_weight, log_weights[i]);
    }

    // Relative to the best particle (best = 1) so the product never underflows.
    particle_weight.clear();
    particle_weight.reserve(particles.size());
    double total_weight = 0.0;
    for (int i = 0; i < particle_count; ++i)
    {
        MEASUREMENT measurement;
        measurement.NUM = i;
        measurement.WEIGHT = std::exp(log_weights[i] - max_log_weight);
        total_weight += measurement.WEIGHT;
        particle_weight.push_back(measurement);
    }

    // Geometric-mean likelihood per observation of the best particle, in (0, 1].
    double confidence_sum = 0.0;
    for (const auto& observation : vision_point_vect)
    {
        confidence_sum += observation.CONFIDENCE;
    }
    double best_fit = 0.0;
    if (confidence_sum > 0.0)
    {
        best_fit = std::exp(max_log_weight / confidence_sum);
    }

    std::sort(particle_weight.begin(), particle_weight.end(), sort_return);

    scored_particles = particles;
    scored_weights.assign(particles.size(), 0.0);
    for (const auto& measurement : particle_weight)
    {
        scored_weights[measurement.NUM] = measurement.WEIGHT;
    }

    if (total_weight > 0 && best_fit > input.min_fit)
    {
        std::printf("[LocalNode] %sMATCHING!! fit %f\n",
                    small_matching ? "SMALL " : "",
                    best_fit);

        const int top_n = 1;
        double sum_w = 0.0;
        for (int i = 0; i < top_n; ++i)
        {
            sum_w += particle_weight[i].WEIGHT;
        }

        double wx = 0.0;
        double wy = 0.0;
        for (int i = 0; i < top_n; ++i)
        {
            const double normalized_weight =
                particle_weight[i].WEIGHT / sum_w;
            wx += normalized_weight * particles[particle_weight[i].NUM].x;
            wy += normalized_weight * particles[particle_weight[i].NUM].y;
        }

        // Gates are in meters with per-axis scales; x/y px per meter differ on LARGE.
        double var_x = 0.0;
        double var_y = 0.0;
        for (int i = 0; i < top_n; ++i)
        {
            const double dx = (particles[particle_weight[i].NUM].x - wx) / FieldCoord::PX_PER_M_X;
            const double dy = (particles[particle_weight[i].NUM].y - wy) / FieldCoord::PX_PER_M_Y;
            var_x += dx * dx;
            var_y += dy * dy;
        }
        var_x /= top_n;
        var_y /= top_n;
        const double std_m = std::sqrt(var_x + var_y);

        const double jump_m = std::hypot(
            (wx - input.robot_x) / FieldCoord::PX_PER_M_X,
            (wy - input.robot_y) / FieldCoord::PX_PER_M_Y);
        if (jump_m >= MIN_JUMP_M && jump_m < MAX_JUMP_M &&
            std_m < MAX_SPREAD_M)
        {
            result.robot_x = static_cast<float>(wx);
            result.robot_y = static_cast<float>(wy);
            result.pose_updated = true;
        }

        const double resample_noise = input.noise_enabled
            ? input.particle_range * 0.1
            : 0.0;
        Resampler::systematic(
            particles, particle_weight, total_weight, resample_noise);
    }
    else
    {
        std::printf("FAIL!! fit %f\n", best_fit);
    }

    appendSyntheticObservations(input, result);

    return result;
}

}
