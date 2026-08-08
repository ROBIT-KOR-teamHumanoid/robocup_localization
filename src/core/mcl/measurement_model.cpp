#include "robocup_localization/core/mcl/measurement_model.hpp"

#include <algorithm>
#include <cmath>

namespace robocup_localization::core
{

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    return x.WEIGHT > y.WEIGHT;
}

double MeasurementModel::particleWeight(
    const Particle& particle,
    const std::vector<VISION_POINT>& observations,
    const PreFilter& filter,
    const MeasurementContext& context)
{
    // 파티클 위치는 픽셀 격자 기준으로 다룬다 (기존 동작 유지: float → int 절단).
    const int   PT_X = static_cast<int>(particle.x);
    const int   PT_Y = static_cast<int>(particle.y);
    const float PT_Z = particle.yaw_deg;

    double weight = 0.0;

    const double pt_z_rad = PT_Z * DEG2RAD;
    const double cp = std::cos(pt_z_rad);
    const double sp = std::sin(pt_z_rad);

    for (std::size_t i = 0; i < observations.size(); i++)
    {
        const VISION_POINT& observation = observations[i];

        // 파티클 θ로 관측 벡터 재변환 (robot-frame mm → world-frame pixel)
        const double raw_x_m = observation.RAW_X_MM / 1000.0;
        const double raw_y_m = observation.RAW_Y_MM / 1000.0;
        const int vec_x = (int)((-raw_x_m * cp - raw_y_m * sp) * FieldCoord::PX_PER_M_X);
        const int vec_y = (int)(( raw_x_m * sp - raw_y_m * cp) * FieldCoord::PX_PER_M_Y);

        const int ptx = PT_X + vec_x;
        const int pty = PT_Y + vec_y;

        double min_dis = LARGE_DIST;
        int    best_j  = -1;

        // bearing: 파티클 θ 기준 재변환된 vec_x, vec_y로 계산
        const double obs_angle = std::atan2(vec_y, vec_x);

        const double obs_feat_field_x = VisionPointUtils::worldX(observation);
        const double obs_feat_field_y = VisionPointUtils::worldY(observation);
        const double obs_ball_angle   =
            std::atan2(obs_feat_field_y - context.ball_field_px_y,
                       obs_feat_field_x - context.ball_field_px_x);
        double best_ball_angle_diff = LARGE_DIST;

        for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
        {
            if (!FieldCoord::ALL_POINTS[j].active)
                continue;
            if (FieldCoord::ALL_POINTS[j].type != (FieldCoord::FeatureType)observation.FEATURE_TYPE)
                continue;

            if (context.robot_y_sign != 0 &&
                observation.FEATURE_TYPE == (int)FieldCoord::L_SHAPE)
            {
                if (FieldCoord::ALL_POINTS[j].y * context.robot_y_sign < 0.0)
                    continue;
            }

            const double exp_angle =
                std::atan2(filter.Local_point_y[j] - PT_Y, filter.Local_point_x[j] - PT_X);
            if (!PreFilter::passBearing(obs_angle, exp_angle))
                continue;

            const double dx_m = (filter.Local_point_x[j] - ptx) / FieldCoord::PX_PER_M_X;
            const double dy_m = (filter.Local_point_y[j] - pty) / FieldCoord::PX_PER_M_Y;
            const double dis  = std::sqrt(dx_m * dx_m + dy_m * dy_m);

            if ((FieldCoord::FeatureType)observation.FEATURE_TYPE == FieldCoord::L_SHAPE)
            {
                const double actual_dist_m =
                    std::sqrt(std::pow((filter.Local_point_x[j] - PT_X) / FieldCoord::PX_PER_M_X, 2)
                            + std::pow((filter.Local_point_y[j] - PT_Y) / FieldCoord::PX_PER_M_Y, 2));
                const double dist_diff =
                    std::fabs(actual_dist_m - observation.DISTANCE / 1000.0);
                if (dist_diff > L_DIST_GATE_M)
                    continue;
            }

            if (context.ball_visible)
            {
                const double exp_ball_angle =
                    std::atan2(filter.Local_point_y[j] - context.ball_field_px_y,
                               filter.Local_point_x[j] - context.ball_field_px_x);
                double ball_angle_diff = std::fabs(obs_ball_angle - exp_ball_angle);
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
        const double err_x_px = ptx - filter.Local_point_x[best_j];
        const double err_y_px = pty - filter.Local_point_y[best_j];

        // 파티클 → 랜드마크 방향 단위벡터
        const double vx      = filter.Local_point_x[best_j] - PT_X;
        const double vy      = filter.Local_point_y[best_j] - PT_Y;
        const double dist_px = std::max(1e-6, std::sqrt(vx * vx + vy * vy));
        const double ux      = vx / dist_px;
        const double uy      = vy / dist_px;

        // normal(dn): 방향 따라, perp(dp): 수직
        const double dn_m = (ux * err_x_px + uy * err_y_px) / FieldCoord::PX_PER_M_X;
        const double dp_m = (-uy * err_x_px + ux * err_y_px) / FieldCoord::PX_PER_M_X;

        // 거리 비례 표준편차 (sigma_min = 0.05m)
        const double dist_m  = dist_px / FieldCoord::PX_PER_M_X;
        const double sigma_n = INV_NORM_VAR * dist_m + 0.05;
        const double sigma_p = INV_PERP_VAR * dist_m + 0.05;

        const double D2 =
            (dn_m * dn_m) / (sigma_n * sigma_n) + (dp_m * dp_m) / (sigma_p * sigma_p);
        const double likelihood = std::exp(-0.5 * D2);

        weight += observation.CONFIDENCE * likelihood;
    }

    return weight;
}

}  // namespace robocup_localization::core
