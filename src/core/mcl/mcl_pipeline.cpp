#include "robocup_localization/core/mcl/mcl_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "robocup_localization/core/mcl/random_util.hpp"

namespace robocup_localization::core
{

void MclPipeline::clearObservations()
{
    vision_point_vect.clear();
    particle_weight.clear();
    vision_data_count = 0;
    vision_data_size  = 0;
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

// 시야 안에 있는데 이번에 안 잡힌 랜드마크를 가짜 관측으로 만들어 다음
// 사이클에 넣는다. 관측이 부족할 때 파티클이 흩어지는 걸 막는 장치.
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
    vision_data_size  = 0;

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

    // 관측 대비 매칭이 빈약하면 파티클을 로봇 주변으로 다시 뿌린다.
    const bool small_matching =
        vision_data_count > 0 &&
        vision_data_size / vision_data_count <= 1;

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

    // ── 2단계: 가중치 계산 ─────────────────────────────────
    particle_weight.clear();
    particle_weight.reserve(particles.size());
    for (int i = 0; i < particle_count; ++i)
    {
        MEASUREMENT measurement;
        measurement.NUM = i;
        measurement.WEIGHT = MeasurementModel::particleWeight(
            particles[i], vision_point_vect, filter, context);
        particle_weight.push_back(measurement);
    }

    double total_weight = 0.0;
    for (const auto& measurement : particle_weight)
    {
        total_weight += measurement.WEIGHT;
    }

    std::sort(particle_weight.begin(), particle_weight.end(), sort_return);

    if (total_weight > 0 &&
        particle_weight.front().WEIGHT > input.weight_update_threshold)
    {
        std::printf("[LocalNode] %sMATCHING!! : %f\n",
                    small_matching ? "SMALL " : "",
                    particle_weight.front().WEIGHT);

        // ── 포즈 추정: 상위 N개 가중평균 ────────────────────
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

        double var_x = 0.0;
        double var_y = 0.0;
        for (int i = 0; i < top_n; ++i)
        {
            const double dx = particles[particle_weight[i].NUM].x - wx;
            const double dy = particles[particle_weight[i].NUM].y - wy;
            var_x += dx * dx;
            var_y += dy * dy;
        }
        var_x /= top_n;
        var_y /= top_n;
        const double std_px = std::sqrt(var_x + var_y);

        // ── 채택 게이트: 너무 작은/큰 점프, 너무 퍼진 분포는 reject ──
        const double jump = std::sqrt(
            std::pow(wx - input.robot_x, 2) +
            std::pow(wy - input.robot_y, 2));
        if (jump >= MIN_JUMP_PX && jump < MAX_JUMP_PX &&
            std_px < MAX_SPREAD_PX)
        {
            result.robot_x = static_cast<float>(wx);
            result.robot_y = static_cast<float>(wy);
            result.pose_updated = true;
        }

        // ── 3단계: 리샘플링 ────────────────────────────────
        const double resample_noise = input.noise_enabled
            ? input.particle_range * 0.1
            : 0.0;
        Resampler::systematic(
            particles, particle_weight, total_weight, resample_noise);
    }
    else
    {
        std::printf("FAIL!! : %f\n", particle_weight.front().WEIGHT);
    }

    appendSyntheticObservations(input, result);

    return result;
}

}  // namespace robocup_localization::core
