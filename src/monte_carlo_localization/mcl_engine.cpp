#include "robot_config.h"
#include "monte_carlo_localization/mcl_engine.hpp"
#include "pre_processing/vision_processing.hpp"
#include "monte_carlo_localization/mcl_node.hpp"

#include <random>

static std::mt19937 &getRng()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}


void MclEngine::featureCalc(MclNode &mcl)
{
    // 락 순서: vision -> pt -> robot0 (mcl_node.hpp 참조).
    // pt 는 예전엔 호출자(mclProcessing)가 잡았는데, 그러면 pt -> vision 이 되어
    // 다른 경로의 vision -> robot0 -> pt 와 순환을 이뤘다. 여기서 순서대로 잡는다.
    std::lock_guard<std::mutex> lock(mcl.vision_mutex);
    std::lock_guard<std::mutex> pt_lock(mcl.pt_mutex);

    if (!mcl.valid_observation)
    {
        vision_point_vect.clear();
        mcl.vision_data_cnt  = 0;
        mcl.vision_data_size = 0;
        return;
    }
    mcl.valid_observation = false;

    obs_size = (int)vision_point_vect.size();

    if (obs_size > 10)
    {
         cout << "PARTICLE UPDATE!! : " << endl;
         
        // 공 필드 좌표 계산 (Ball_2d_X/Y = pan-rotated robot-relative mm)
        float robot0_x, robot0_y, robot0_z;
        {
            std::lock_guard<std::mutex> robot0_lock(mcl.robot0_mutex);
            robot0_x = mcl.robot0.x;
            robot0_y = mcl.robot0.y;
            robot0_z = mcl.robot0.yaw_deg;
        }
        const double ball_mm_x = mcl.ball_mm_x;
        const double ball_mm_y = mcl.ball_mm_y;
        ball_visible = (ball_mm_x != 0.0 || ball_mm_y != 0.0);

        const auto ball_px = VisionProcessing::toFieldPx(
            ball_mm_x, ball_mm_y, {robot0_x, robot0_y, robot0_z});
        ball_field_px_x = ball_px.x;
        ball_field_px_y = ball_px.y;

        if (mcl.vision_data_cnt > 0 && mcl.vision_data_size / mcl.vision_data_cnt <= 1)
        {
            for (int i = 0; i < mcl.particle_num; i++)
                random_point(mcl.pt[i], robot0_x, robot0_y, mcl.particle_range, robot0_z);
        }

        filter.setCircle(robot0_z, vision_point_vect);

        filter.prefilter(vision_point_vect);

        for (int i = 0; i < mcl.particle_num; i++)
        {
            fc_meas.NUM    = i;
            fc_meas.WEIGHT = weight_calculate(mcl.pt[i].x, mcl.pt[i].y, mcl.pt[i].yaw_deg,
                                              robot0_x, robot0_y,
                                              vision_point_vect);
            particle_weight.push_back(fc_meas);
        }

        total_weight = 0.0;
        for (auto &m : particle_weight)
            total_weight += m.WEIGHT;

        sort(particle_weight.begin(), particle_weight.end(), sort_return);

        if (total_weight > 0 && particle_weight[0].WEIGHT > mcl.weight_update_threshold)
        {
            if (mcl.vision_data_cnt > 0 && mcl.vision_data_size / mcl.vision_data_cnt <= 1)
                printf("[LocalNode] SMALL MATCHING!! : %f\n", particle_weight[0].WEIGHT);
            else
                printf("[LocalNode] MATCHING!! : %f\n", particle_weight[0].WEIGHT);

            top_n = 1;  
            sum_w = 0.0;
            wx    = 0.0;
            wy    = 0.0;
            for (int i = 0; i < top_n; i++)
                sum_w += particle_weight[i].WEIGHT;
            for (int i = 0; i < top_n; i++)
            {
                double w = particle_weight[i].WEIGHT / sum_w;
                wx += w * mcl.pt[particle_weight[i].NUM].x;
                wy += w * mcl.pt[particle_weight[i].NUM].y;
            }
            // top-5 파티클 위치 분산 계산 - 너무 퍼져있으면 신뢰할 수 없는 매칭
            double var_x = 0.0, var_y = 0.0;
            for (int i = 0; i < top_n; i++)
            {
                double dx = mcl.pt[particle_weight[i].NUM].x - wx;
                double dy = mcl.pt[particle_weight[i].NUM].y - wy;
                var_x += dx * dx;
                var_y += dy * dy;
            }
            var_x /= top_n;
            var_y /= top_n;
            double std_px = sqrt(var_x + var_y);

            double jump = sqrt(pow(wx - robot0_x, 2) + pow(wy - robot0_y, 2));
            if (jump >= MclTune::minJumpPx() && jump < MclTune::maxJumpPx() &&
                std_px < MclTune::maxSpreadPx())
            {
                std::lock_guard<std::mutex> robot0_lock(mcl.robot0_mutex);
                mcl.robot0.x = wx;
                mcl.robot0.y = wy;
            }

            cdf.resize(mcl.particle_num);
            cdf[0] = particle_weight[0].WEIGHT / total_weight;
            for (int i = 1; i < mcl.particle_num; i++)
                cdf[i] = cdf[i - 1] + particle_weight[i].WEIGHT / total_weight;

            fc_step = 1.0 / mcl.particle_num;
            fc_r    = std::uniform_real_distribution<double>(0.0, fc_step)(getRng());
            new_x.resize(mcl.particle_num);
            new_y.resize(mcl.particle_num);
            fc_j = 0;
            for (int i = 0; i < mcl.particle_num; i++)
            {
                double target = fc_r + i * fc_step;
                while (fc_j < mcl.particle_num - 1 && cdf[fc_j] < target)
                    fc_j++;
                new_x[i] = mcl.pt[particle_weight[fc_j].NUM].x;
                new_y[i] = mcl.pt[particle_weight[fc_j].NUM].y;
            }

            resample_noise = mcl.noise_enable ? mcl.particle_range * 0.1 : 0.0;
            for (int i = 0; i < mcl.particle_num; i++)
            {
                mcl.pt[i].x = new_x[i] + gaussianRandom(0, resample_noise);
                mcl.pt[i].y = new_y[i] + gaussianRandom(0, resample_noise);
            }
        }
        else
        {
            cout << "FAIL!! : " << particle_weight[0].WEIGHT << endl;
        }

        synthetic_cnt = (int)vision_point_vect.size() - mcl.vision_data_cnt;
        if (synthetic_cnt < 0)
            synthetic_cnt = 0;

        // 실제 관측된 타입만 수집 (clear 전)
        bool observed_types[3] = {false, false, false};
        for (int k = synthetic_cnt; k < (int)vision_point_vect.size(); k++)
        {
            int t = vision_point_vect[k].FEATURE_TYPE;
            if (t >= 0 && t < 3)
                observed_types[t] = true;
        }

        // 위 매칭 블록에서 robot0.x/y 가 갱신됐을 수 있으므로 다시 스냅샷한다.
        // (synthetic 관측은 보정된 pose 기준이어야 한다)
        {
            std::lock_guard<std::mutex> robot0_lock(mcl.robot0_mutex);
            robot0_x = mcl.robot0.x;
            robot0_y = mcl.robot0.y;
            robot0_z = mcl.robot0.yaw_deg;
        }

        filter.checkLocalPoint(synthetic_cnt, robot0_z, vision_point_vect);

        particle_weight.clear();
        vision_point_vect.clear();
        mcl.vision_data_cnt  = 0;
        mcl.vision_data_size = 0;

        for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
        {
            if (!FieldCoord::ALL_POINTS[i].active)
                continue;
            if (filter.Local_point_check[i] != 1)
                continue;
            int feat_type = static_cast<int>(FieldCoord::ALL_POINTS[i].type);
            if (feat_type < 0 || feat_type >= 3 || !observed_types[feat_type])
                continue;
            if (robot_y_sign != 0 && feat_type == (int)FieldCoord::L_SHAPE)
            {
                if (FieldCoord::ALL_POINTS[i].y * robot_y_sign < 0.0)
                    continue;
            }
            vision_point_vect.push_back(VisionPointUtils::makeSynthetic(
                filter.Local_point_x[i], filter.Local_point_y[i],
                robot0_x, robot0_y,
                feat_type,
                robot0_z));
        }
    }
}

double MclEngine::weight_calculate(double PT_X, double PT_Y, float PT_Z, double NOW_X, double NOW_Y, vector<VISION_POINT> &vect)
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
        double vec_x = (-raw_x_m * cp - raw_y_m * sp) * FieldCoord::PX_PER_M;
        double vec_y = ( raw_x_m * sp - raw_y_m * cp) * FieldCoord::PX_PER_M;

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

            double dx_m = (filter.Local_point_x[j] - ptx) / FieldCoord::PX_PER_M;
            double dy_m = (filter.Local_point_y[j] - pty) / FieldCoord::PX_PER_M;
            double dis  = sqrt(dx_m * dx_m + dy_m * dy_m);

            if ((FieldCoord::FeatureType)vect[i].FEATURE_TYPE == FieldCoord::L_SHAPE)
            {
                double actual_dist_m = sqrt(pow((filter.Local_point_x[j] - PT_X) / FieldCoord::PX_PER_M, 2)
                                          + pow((filter.Local_point_y[j] - PT_Y) / FieldCoord::PX_PER_M, 2));
                double dist_diff = fabs(actual_dist_m - vect[i].DISTANCE / 1000.0);
                if (dist_diff > MclTune::L_DIST_GATE_M)
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
        double dn_m = (ux * err_x_px + uy * err_y_px) / FieldCoord::PX_PER_M;
        double dp_m = (-uy * err_x_px + ux * err_y_px) / FieldCoord::PX_PER_M;

        // 거리 비례 표준편차 (sigma_min = 0.05m)
        double dist_m   = dist_px / FieldCoord::PX_PER_M;
        double sigma_n  = INV_NORM_VAR * dist_m + 0.05;
        double sigma_p  = INV_PERP_VAR * dist_m + 0.05;

        double D2       = (dn_m * dn_m) / (sigma_n * sigma_n) + (dp_m * dp_m) / (sigma_p * sigma_p);
        double likelihood = exp(-0.5 * D2);

        weight += vect[i].CONFIDENCE * likelihood;
    }
    return weight;
}

double MclEngine::gaussianRandom(double average, double stdev)
{
    static std::normal_distribution<double> dist(0.0, 1.0);
    return average + stdev * dist(getRng());
}

void MclEngine::random_point(Particle &p, int x, int y, float particle_range, float theta)
{
    p.x = x + gaussianRandom(0, particle_range);
    p.y = y + gaussianRandom(0, particle_range);
    p.yaw_deg = theta + gaussianRandom(0, THETA_INIT_SPREAD_DEG);
}

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    return x.WEIGHT > y.WEIGHT;
}
