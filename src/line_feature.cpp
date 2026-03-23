#include "../../robot_config.h"
#include "../include/objects/line_feature.hpp"
#include "../include/robocup_localization/qnode.hpp"


void Line_Feature::featureCalc(QNode *qnode, double particle_range)
{
    std::lock_guard<std::mutex> lock(qnode->vision_mutex);

    obs_size = (int)vision_point_vect.size();

    if (obs_size > 10)
    {
        cout << "particle update" << endl;
        if (qnode->vision_data_cnt > 0 && qnode->vision_data_size / qnode->vision_data_cnt <= 1)
        {
            for (int i = 0; i < qnode->particle_num; i++)
                random_point(qnode->pt[i], qnode->robot0.x, qnode->robot0.y, particle_range);
        }

        // 공 필드 좌표 계산 (Ball_2d_X/Y = pan-rotated robot-relative mm)
        double yaw_rad      = qnode->robot0.z * M_PI / 180.0;
        double ball_mm_x    = qnode->visionMSG.Ball_2d_X;
        double ball_mm_y    = qnode->visionMSG.Ball_2d_Y;
        ball_visible        = (ball_mm_x != 0.0 || ball_mm_y != 0.0);
        ball_field_px_x     = qnode->robot0.x + (-1) * (ball_mm_x / 1000.0 * FieldCoord::PX_PER_M_X * cos(yaw_rad) + ball_mm_y / 1000.0 * FieldCoord::PX_PER_M_X * sin(yaw_rad));
        ball_field_px_y     = qnode->robot0.y + (ball_mm_x / 1000.0 * FieldCoord::PX_PER_M_Y * sin(yaw_rad) - ball_mm_y / 1000.0 * FieldCoord::PX_PER_M_Y * cos(yaw_rad));

        filter.setCircle(qnode->robot0.z, vision_point_vect);

        filter.prefilter(vision_point_vect);

        for (int i = 0; i < qnode->particle_num; i++)
        {
            fc_meas.NUM    = i;
            fc_meas.WEIGHT = weight_calculate(qnode->pt[i].x, qnode->pt[i].y,
                                              qnode->robot0.x, qnode->robot0.y,
                                              vision_point_vect);
            particle_weight.push_back(fc_meas);
        }

        total_weight = 0.0;
        for (auto &m : particle_weight)
            total_weight += m.WEIGHT;

        sort(particle_weight.begin(), particle_weight.end(), sort_return);

        if (total_weight > 0 && particle_weight[0].WEIGHT > qnode->weight_update_threshold)
        {
            if (qnode->vision_data_cnt > 0 && qnode->vision_data_size / qnode->vision_data_cnt <= 1)
                cout << "SMALL MATCHING!! : " << particle_weight[0].WEIGHT << endl;
            else
                cout << "MATCHING!! : " << particle_weight[0].WEIGHT << endl;

            top_n = 5;
            sum_w = 0.0;
            wx    = 0.0;
            wy    = 0.0;
            for (int i = 0; i < top_n; i++)
                sum_w += particle_weight[i].WEIGHT;
            for (int i = 0; i < top_n; i++)
            {
                double w = particle_weight[i].WEIGHT / sum_w;
                wx += w * qnode->pt[particle_weight[i].NUM].x;
                wy += w * qnode->pt[particle_weight[i].NUM].y;
            }
            double jump = sqrt(pow(wx - qnode->robot0.x, 2) + pow(wy - qnode->robot0.y, 2));
            if (jump < MAX_JUMP_PX)
            {
                qnode->robot0.x = (int)wx;
                qnode->robot0.y = (int)wy;
            }

            cdf.resize(qnode->particle_num);
            cdf[0] = particle_weight[0].WEIGHT / total_weight;
            for (int i = 1; i < qnode->particle_num; i++)
                cdf[i] = cdf[i - 1] + particle_weight[i].WEIGHT / total_weight;

            fc_step = 1.0 / qnode->particle_num;
            fc_r    = ((double)rand() / RAND_MAX) * fc_step;
            new_x.resize(qnode->particle_num);
            new_y.resize(qnode->particle_num);
            fc_j = 0;
            for (int i = 0; i < qnode->particle_num; i++)
            {
                double target = fc_r + i * fc_step;
                while (fc_j < qnode->particle_num - 1 && cdf[fc_j] < target)
                    fc_j++;
                new_x[i] = qnode->pt[particle_weight[fc_j].NUM].x;
                new_y[i] = qnode->pt[particle_weight[fc_j].NUM].y;
            }

            resample_noise = particle_range * 0.1;
            for (int i = 0; i < qnode->particle_num; i++)
            {
                qnode->pt[i].x = new_x[i] + gaussianRandom(0, resample_noise);
                qnode->pt[i].y = new_y[i] + gaussianRandom(0, resample_noise);
            }
        }
        else
        {
            cout << "FAIL!! : " << particle_weight[0].WEIGHT << endl;
        }

        synthetic_cnt = (int)vision_point_vect.size() - qnode->vision_data_cnt;
        if (synthetic_cnt < 0)
            synthetic_cnt = 0;

        filter.checkLocalPoint(synthetic_cnt, qnode->robot0.z, vision_point_vect);

        particle_weight.clear();
        vision_point_vect.clear();
        qnode->vision_data_cnt  = 0;
        qnode->vision_data_size = 0;

        for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
        {
            if (!FieldCoord::ALL_POINTS[i].active)
                continue;
            if (filter.Local_point_check[i] == 1)
            {
                vision_point_vect.push_back(VisionPointUtils::makeSynthetic(
                    filter.Local_point_x[i], filter.Local_point_y[i],
                    qnode->robot0.x, qnode->robot0.y,
                    static_cast<int>(FieldCoord::ALL_POINTS[i].type)));
            }
        }
    }
}

double Line_Feature::weight_calculate(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
{
    // !!!!!!!제일 중요한 부분!!!!!!!
    // PRE CONDITION : PT_X, PT_Y, ROBOT_X, ROBOT_Y, VECTOR<VISION_POINT> = VISION_POINT_VECT
    // POST CONDITION : weight
    // PURPOSE : 가중치를 구하는 함수

    weight = 0.0;

    for (int i = 0; i < (int)vect.size(); i++)
    {
        ptx = PT_X + VisionPointUtils::worldX(vect[i]) - NOW_X;
        pty = PT_Y + VisionPointUtils::worldY(vect[i]) - NOW_Y;

        min_dis = LARGE_DIST;

        // 관측 방향 (세계 좌표계, POINT_VEC은 이미 yaw 변환됨)
        obs_angle = VisionPointUtils::bearing(vect[i]);

        // 공 기준 관측 각도 (필드 좌표계)
        double obs_feat_field_x = VisionPointUtils::worldX(vect[i]);
        double obs_feat_field_y = VisionPointUtils::worldY(vect[i]);
        double obs_ball_angle   = atan2(obs_feat_field_y - ball_field_px_y,
                                        obs_feat_field_x - ball_field_px_x);

        double best_ball_angle_diff = LARGE_DIST;

        // X_SHAPE: 파티클↔랜드마크 실제 거리와 관측 거리 차이가 최소인 랜드마크 하나만 사용
        int nearest_x_idx = -1;
        if ((FieldCoord::FeatureType)vect[i].FEATURE_TYPE == FieldCoord::X_SHAPE)
        {
            double obs_dist_m  = vect[i].DISTANCE / 1000.0;
            double best_x_diff = LARGE_DIST;
            for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
            {
                if (!FieldCoord::ALL_POINTS[j].active)
                    continue;
                if (FieldCoord::ALL_POINTS[j].type != FieldCoord::X_SHAPE)
                    continue;
                double dx_m       = (filter.Local_point_x[j] - PT_X) / FieldCoord::PX_PER_M_X;
                double dy_m       = (filter.Local_point_y[j] - PT_Y) / FieldCoord::PX_PER_M_Y;
                double actual_dist = sqrt(dx_m * dx_m + dy_m * dy_m);
                double diff        = fabs(actual_dist - obs_dist_m);
                if (diff < best_x_diff)
                {
                    best_x_diff   = diff;
                    nearest_x_idx = j;
                }
            }
        }

        for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
        {
            if (!FieldCoord::ALL_POINTS[j].active)
                continue;
            if (FieldCoord::ALL_POINTS[j].type != (FieldCoord::FeatureType)vect[i].FEATURE_TYPE)
                continue;

            // X_SHAPE: 가장 가까운 랜드마크만 사용
            if ((FieldCoord::FeatureType)vect[i].FEATURE_TYPE == FieldCoord::X_SHAPE
                && j != nearest_x_idx)
                continue;

            // bearing 필터
            exp_angle = atan2(filter.Local_point_y[j] - PT_Y, filter.Local_point_x[j] - PT_X);
            if (!Filter::passBearing(obs_angle, exp_angle))
                continue;

            double dx_m = (filter.Local_point_x[j] - ptx) / FieldCoord::PX_PER_M_X;
            double dy_m = (filter.Local_point_y[j] - pty) / FieldCoord::PX_PER_M_Y;
            double dis  = sqrt(dx_m * dx_m + dy_m * dy_m);

            if (ball_visible)
            {
                // 공 기준 각도가 가장 잘 맞는 랜드마크 선택
                double exp_ball_angle = atan2(filter.Local_point_y[j] - ball_field_px_y,
                                              filter.Local_point_x[j] - ball_field_px_x);
                double ball_angle_diff = fabs(obs_ball_angle - exp_ball_angle);
                if (ball_angle_diff > M_PI)
                    ball_angle_diff = 2 * M_PI - ball_angle_diff;

                if (ball_angle_diff < best_ball_angle_diff)
                {
                    best_ball_angle_diff = ball_angle_diff;
                    min_dis = dis;
                }
            }
            else
            {
                // 공 미감지 시 기존 방식 (최소 거리)
                if (dis < min_dis)
                    min_dis = dis;
            }
        }

        if (min_dis <= MIN_DIS_CLAMP)
            min_dis = MIN_DIS_CLAMP;

        weight += (WEIGHT_SCALE / min_dis) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / DIST_NORM);
    }
    return weight;
}

double Line_Feature::gaussianRandom(double average, double stdev)
{
    double v1, v2, s, temp;
    do
    {
        v1 = 2 * ((double)rand() / RAND_MAX) - 1;
        v2 = 2 * ((double)rand() / RAND_MAX) - 1;
        s  = v1 * v1 + v2 * v2;
    } while (s >= 1 || s == 0);
    s    = sqrt((-2 * log(s)) / s);
    temp = v1 * s;
    temp = (stdev * temp) + average;
    return temp;
}

void Line_Feature::random_point(ROBOT &robot, int x, int y, float particle_range)
{
    robot.x = x + gaussianRandom(0, particle_range);
    robot.y = y + gaussianRandom(0, particle_range);
}

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    return x.WEIGHT > y.WEIGHT;
}
