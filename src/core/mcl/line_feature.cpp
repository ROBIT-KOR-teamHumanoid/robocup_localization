#include "robot_config.h"
#include "robocup_localization/core/mcl/line_feature.hpp"

#include <algorithm>
#include <random>

static std::mt19937 &getRng()
{
    static thread_local std::mt19937 gen(std::random_device{}());
    return gen;
}


FeatureCalcResult Line_Feature::featureCalc(
    std::vector<Particle>& particles,
    const FeatureCalcInput& input)
{
    FeatureCalcResult result;
    result.robot_x = input.robot_x;
    result.robot_y = input.robot_y;

    if (!valid_observation)
    {
        vision_point_vect.clear();
        particle_weight.clear();
        vision_data_count = 0;
        vision_data_size = 0;
        return result;
    }
    valid_observation = false;

    const int particle_count = static_cast<int>(particles.size());
    if (particle_count == 0)
    {
        vision_point_vect.clear();
        particle_weight.clear();
        vision_data_count = 0;
        vision_data_size = 0;
        return result;
    }

    obs_size = static_cast<int>(vision_point_vect.size());

    if (obs_size > 10)
    {
        cout << "PARTICLE UPDATE!! : " << endl;

        const double yaw_rad = input.robot_yaw_deg * M_PI / 180.0;
        ball_visible =
            ball_relative_x_mm != 0.0 || ball_relative_y_mm != 0.0;
        ball_field_px_x = input.robot_x + (-1) *
            (ball_relative_x_mm / 1000.0 * FieldCoord::PX_PER_M_X * cos(yaw_rad) +
             ball_relative_y_mm / 1000.0 * FieldCoord::PX_PER_M_X * sin(yaw_rad));
        ball_field_px_y = input.robot_y +
            (ball_relative_x_mm / 1000.0 * FieldCoord::PX_PER_M_Y * sin(yaw_rad) -
             ball_relative_y_mm / 1000.0 * FieldCoord::PX_PER_M_Y * cos(yaw_rad));

        const bool small_matching =
            vision_data_count > 0 &&
            vision_data_size / vision_data_count <= 1;

        if (small_matching)
        {
            for (auto& particle : particles)
            {
                random_point(
                    particle, input.robot_x, input.robot_y,
                    input.particle_range, input.robot_yaw_deg);
            }
        }

        filter.setCircle(input.robot_yaw_deg, vision_point_vect);
        filter.prefilter(vision_point_vect);

        particle_weight.clear();
        particle_weight.reserve(particles.size());
        for (int i = 0; i < particle_count; ++i)
        {
            fc_meas.NUM = i;
            fc_meas.WEIGHT = weight_calculate(
                particles[i].x, particles[i].y, particles[i].z,
                input.robot_x, input.robot_y, vision_point_vect);
            particle_weight.push_back(fc_meas);
        }

        total_weight = 0.0;
        for (const auto& measurement : particle_weight)
        {
            total_weight += measurement.WEIGHT;
        }

        sort(particle_weight.begin(), particle_weight.end(), sort_return);

        if (total_weight > 0 &&
            particle_weight.front().WEIGHT > input.weight_update_threshold)
        {
            if (small_matching)
            {
                printf("[LocalNode] SMALL MATCHING!! : %f\n",
                       particle_weight.front().WEIGHT);
            }
            else
            {
                printf("[LocalNode] MATCHING!! : %f\n",
                       particle_weight.front().WEIGHT);
            }

            top_n = 1;
            sum_w = 0.0;
            wx = 0.0;
            wy = 0.0;
            for (int i = 0; i < top_n; ++i)
            {
                sum_w += particle_weight[i].WEIGHT;
            }
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
            const double std_px = sqrt(var_x + var_y);

            const double jump = sqrt(
                pow(wx - input.robot_x, 2) +
                pow(wy - input.robot_y, 2));
            if (jump >= MIN_JUMP_PX && jump < MAX_JUMP_PX &&
                std_px < MAX_SPREAD_PX)
            {
                result.robot_x = static_cast<float>(wx);
                result.robot_y = static_cast<float>(wy);
                result.pose_updated = true;
            }

            cdf.resize(particle_count);
            cdf[0] = particle_weight[0].WEIGHT / total_weight;
            for (int i = 1; i < particle_count; ++i)
            {
                cdf[i] = cdf[i - 1] +
                    particle_weight[i].WEIGHT / total_weight;
            }

            fc_step = 1.0 / particle_count;
            fc_r = std::uniform_real_distribution<double>(
                0.0, fc_step)(getRng());
            new_x.resize(particle_count);
            new_y.resize(particle_count);
            new_z.resize(particle_count);
            fc_j = 0;
            for (int i = 0; i < particle_count; ++i)
            {
                const double target = fc_r + i * fc_step;
                while (fc_j < particle_count - 1 && cdf[fc_j] < target)
                {
                    ++fc_j;
                }
                const Particle& selected =
                    particles[particle_weight[fc_j].NUM];
                new_x[i] = selected.x;
                new_y[i] = selected.y;
                new_z[i] = selected.z;
            }

            resample_noise = input.noise_enabled
                ? input.particle_range * 0.1
                : 0.0;
            for (int i = 0; i < particle_count; ++i)
            {
                particles[i].x =
                    new_x[i] + gaussianRandom(0, resample_noise);
                particles[i].y =
                    new_y[i] + gaussianRandom(0, resample_noise);
                particles[i].z = new_z[i];
            }
        }
        else
        {
            cout << "FAIL!! : " << particle_weight.front().WEIGHT << endl;
        }

        synthetic_cnt =
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

        filter.checkLocalPoint(
            synthetic_cnt, input.robot_yaw_deg, vision_point_vect);

        particle_weight.clear();
        vision_point_vect.clear();
        vision_data_count = 0;
        vision_data_size = 0;

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

    return result;
}

double Line_Feature::weight_calculate(int PT_X, int PT_Y, float PT_Z, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
{
    // PRE CONDITION : PT_X, PT_Y, PT_Z(파티클 yaw deg), ROBOT_X, ROBOT_Y, VECTOR<VISION_POINT>
    // POST CONDITION : weight
    // PURPOSE : Mahalanobis distance 기반 가중치 계산 (per-particle theta 적용)
    //   관측(RAW_X_MM, RAW_Y_MM)을 파티클 θ로 재변환 → 파티클별 독립 방향 추정
    //   D^2 = dn^2/sigma_n^2 + dp^2/sigma_p^2
    //   weight += CONFIDENCE * exp(-0.5 * D^2)

    weight = 0.0;

    double pt_z_rad = PT_Z * DEG2RAD;
    double cp = cos(pt_z_rad);
    double sp = sin(pt_z_rad);

    for (int i = 0; i < (int)vect.size(); i++)
    {
        // 파티클 θ로 관측 벡터 재변환 (robot-frame mm → world-frame pixel)
        double raw_x_m = vect[i].RAW_X_MM / 1000.0;
        double raw_y_m = vect[i].RAW_Y_MM / 1000.0;
        int vec_x = (int)((-raw_x_m * cp - raw_y_m * sp) * FieldCoord::PX_PER_M_X);
        int vec_y = (int)(( raw_x_m * sp - raw_y_m * cp) * FieldCoord::PX_PER_M_Y);

        ptx = PT_X + vec_x;
        pty = PT_Y + vec_y;

        min_dis = LARGE_DIST;
        best_j  = -1;

        // bearing: 파티클 θ 기준 재변환된 vec_x, vec_y로 계산
        obs_angle = atan2(vec_y, vec_x);

        double obs_feat_field_x = VisionPointUtils::worldX(vect[i]);
        double obs_feat_field_y = VisionPointUtils::worldY(vect[i]);
        double obs_ball_angle   = atan2(obs_feat_field_y - ball_field_px_y,
                                        obs_feat_field_x - ball_field_px_x);
        double best_ball_angle_diff = LARGE_DIST;

        for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
        {
            if (!FieldCoord::ALL_POINTS[j].active)
                continue;
            if (FieldCoord::ALL_POINTS[j].type != (FieldCoord::FeatureType)vect[i].FEATURE_TYPE)
                continue;

            if (robot_y_sign != 0 && vect[i].FEATURE_TYPE == (int)FieldCoord::L_SHAPE)
            {
                if (FieldCoord::ALL_POINTS[j].y * robot_y_sign < 0.0)
                    continue;
            }

            exp_angle = atan2(filter.Local_point_y[j] - PT_Y, filter.Local_point_x[j] - PT_X);
            if (!Filter::passBearing(obs_angle, exp_angle))
                continue;

            double dx_m = (filter.Local_point_x[j] - ptx) / FieldCoord::PX_PER_M_X;
            double dy_m = (filter.Local_point_y[j] - pty) / FieldCoord::PX_PER_M_Y;
            double dis  = sqrt(dx_m * dx_m + dy_m * dy_m);

            if ((FieldCoord::FeatureType)vect[i].FEATURE_TYPE == FieldCoord::L_SHAPE)
            {
                double actual_dist_m = sqrt(pow((filter.Local_point_x[j] - PT_X) / FieldCoord::PX_PER_M_X, 2)
                                          + pow((filter.Local_point_y[j] - PT_Y) / FieldCoord::PX_PER_M_Y, 2));
                double dist_diff = fabs(actual_dist_m - vect[i].DISTANCE / 1000.0);
                if (dist_diff > L_DIST_GATE_M)
                    continue;
            }

            if (ball_visible)
            {
                double exp_ball_angle = atan2(filter.Local_point_y[j] - ball_field_px_y,
                                              filter.Local_point_x[j] - ball_field_px_x);
                double ball_angle_diff = fabs(obs_ball_angle - exp_ball_angle);
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

        // 매칭된 랜드마크 없으면 skip
        if (best_j < 0)
            continue;

        // ── Mahalanobis distance 계산 ──────────────────────────────
        // 오차 벡터: 관측 추정 위치 - 실제 랜드마크 위치 (픽셀)
        double err_x_px = ptx - filter.Local_point_x[best_j];
        double err_y_px = pty - filter.Local_point_y[best_j];

        // 파티클 → 랜드마크 방향 단위벡터
        double vx      = filter.Local_point_x[best_j] - PT_X;
        double vy      = filter.Local_point_y[best_j] - PT_Y;
        double dist_px = max(1e-6, sqrt(vx * vx + vy * vy));
        double ux      = vx / dist_px;
        double uy      = vy / dist_px;

        // normal(dn): 방향 따라, perp(dp): 수직
        double dn_m = (ux * err_x_px + uy * err_y_px) / FieldCoord::PX_PER_M_X;
        double dp_m = (-uy * err_x_px + ux * err_y_px) / FieldCoord::PX_PER_M_X;

        // 거리 비례 표준편차 (sigma_min = 0.05m)
        double dist_m   = dist_px / FieldCoord::PX_PER_M_X;
        double sigma_n  = INV_NORM_VAR * dist_m + 0.05;
        double sigma_p  = INV_PERP_VAR * dist_m + 0.05;

        double D2       = (dn_m * dn_m) / (sigma_n * sigma_n) + (dp_m * dp_m) / (sigma_p * sigma_p);
        double likelihood = exp(-0.5 * D2);

        weight += vect[i].CONFIDENCE * likelihood;
    }
    return weight;
}

double Line_Feature::gaussianRandom(double average, double stdev)
{
    static thread_local std::normal_distribution<double> dist(0.0, 1.0);
    return average + stdev * dist(getRng());
}

void Line_Feature::random_point(
    Particle& particle,
    int x,
    int y,
    float particle_range,
    float theta)
{
    particle.x = x + gaussianRandom(0, particle_range);
    particle.y = y + gaussianRandom(0, particle_range);
    particle.z = theta + gaussianRandom(0, THETA_INIT_SPREAD_DEG);
}

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    return x.WEIGHT > y.WEIGHT;
}
