#include "../../robot_config.h"
#include "../include/objects/line_feature.hpp"
#include "../include/robocup_localization/qnode.hpp"

// ============================================================
// 9-zone 경계 픽셀 단위, FieldCoord 기준 자동 계산
//
//  [  Zone1  |  Zone2  |  Zone3  ]
//  [  Zone4  |  Zone5  |  Zone6  ]
//  [  Zone7  |  Zone8  |  Zone9  ]

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

        set_circle(qnode->robot0.z, vision_point_vect);

        // pre-filter: robot0 기준 estimated world 좌표와 nearest landmark 거리가 OBS_FILTER_DIST_PX 초과인 obs 제거
        {
            auto it = vision_point_vect.begin();
            while (it != vision_point_vect.end())
            {
                int wx = it->STD_X + it->POINT_VEC_X;
                int wy = it->STD_Y + it->POINT_VEC_Y;
                double nearest = LARGE_DIST;
                for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
                {
                    if (FieldCoord::ALL_POINTS[j].type == (FieldCoord::FeatureType)it->FEATURE_TYPE)
                    {
                        double dx_m = (Local_point_x[j] - wx) / FieldCoord::PX_PER_M_X;
                        double dy_m = (Local_point_y[j] - wy) / FieldCoord::PX_PER_M_Y;
                        double d = sqrt(dx_m * dx_m + dy_m * dy_m);
                        if (d < nearest)
                            nearest = d;
                    }
                }
                if (nearest > OBS_FILTER_DIST_M)
                    it = vision_point_vect.erase(it);
                else
                    ++it;
            }
        }

        for (int i = 0; i < qnode->particle_num; i++)
        {
            fc_meas.NUM = i;
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

            top_n = 1;
            sum_w = 0.0;
            wx = 0.0;
            wy = 0.0;
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
            fc_r = ((double)rand() / RAND_MAX) * fc_step;
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

        check_local_point(synthetic_cnt, qnode->robot0.z, vision_point_vect);

        particle_weight.clear();
        vision_point_vect.clear();
        qnode->vision_data_cnt = 0;
        qnode->vision_data_size = 0;

        for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
        {
            if (Local_point_check[i] == 1)
            {
                vision_point.CONFIDENCE = 0.1;
                vision_point.DISTANCE = 1;
                vision_point.POINT_VEC_X = Local_point_x[i] - qnode->robot0.x;
                vision_point.POINT_VEC_Y = Local_point_y[i] - qnode->robot0.y;
                vision_point.STD_X = qnode->robot0.x;
                vision_point.STD_Y = qnode->robot0.y;
                vision_point.FEATURE_TYPE = static_cast<int>(FieldCoord::ALL_POINTS[i].type);
                vision_point_vect.push_back(vision_point);
            }
        }
    }
}

void Line_Feature::compute_circle(vector<VISION_POINT> &vect, int start, double yaw)
{
    // PRE CONDITION : VECTOR<VISION_POINT>, start(순회 시작 인덱스), yaw
    // POST CONDITION : CIRCLE_CENTER, CIRCLE_R
    // PURPOSE : 감지된 특징점들의 외접원 계산 (set_circle / check_local_point 공통 로직)

    X_MAX = -9999;
    X_MIN = 9999;
    Y_MAX = -9999;
    Y_MIN = 9999;

    // 포착된 포인트 중 최대 거리와 최소 거리를 계산
    for (int i = start; i < (int)vect.size(); i++)
    {
        transformation_x = vect[i].POINT_VEC_X * cos(-yaw * DEG2RAD) - vect[i].POINT_VEC_Y * sin(-yaw * DEG2RAD);
        transformation_y = vect[i].POINT_VEC_X * sin(-yaw * DEG2RAD) + vect[i].POINT_VEC_Y * cos(-yaw * DEG2RAD);
        if (transformation_x > X_MAX)
            X_MAX = transformation_x;
        if (transformation_x < X_MIN)
            X_MIN = transformation_x;
        if (transformation_y > Y_MAX)
            Y_MAX = transformation_y;
        if (transformation_y < Y_MIN)
            Y_MIN = transformation_y;
    }

    // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 더한 후 해당 값을 범위의 중앙으로 설정
    CIRCLE_CENTER = Point(
        (int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)),
        (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));

    // 로봇의 좌표에서 최대 거리 값과 최소 거리 값의 평균 값을 뺀 후 해당 값에서 +CIRCLE_MARGIN_PX 한 값을 범위의 반지름으로 설정
    CIRCLE_R = 0;
    for (int i = start; i < (int)vect.size(); i++)
    {
        dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) +
                   pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
        if (CIRCLE_R < dis)
            CIRCLE_R = dis;
    }
    CIRCLE_R += FieldCoord::CIRCLE_MARGIN_PX; // 원 안에 들어오는 특징점만 활성화, UI에 검은색 원으로 표시
}

void Line_Feature::set_circle(double yaw, vector<VISION_POINT> &vect)
{
    // PRE CONDITION : YAW, VECTOR<VISION_POINT>
    // POST CONDITION : CIRCLE_CENTER, CIRCLE_R
    // PURPOSE : 로컬의 특징점 범위를 설정

    if (vect.empty())
        return;
    compute_circle(vect, 0, yaw);
}

void Line_Feature::check_local_point(int ago_point_cnt, double yaw, vector<VISION_POINT> &vect)
{
    // PRE CONDITION : ago_point_cnt = 50, yaw, vect<VISION_POINT> = VISION_POINT_VECT
    // POST CONDITION : Local_point_check
    // PURPOSE : 로봇의 yaw값을 통해 파티클 필터 계산에 사용될 특징점 활성화

    if (vect.empty() || ago_point_cnt >= (int)vect.size())
        return;

    // 이때 벡터 컨테이너에서 50번째 이후 요소부터 계산 (실제 관측분만)
    compute_circle(vect, ago_point_cnt, yaw);

    for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
    {
        if (pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y[i], 2))
            Local_point_check[i] = 1;
        else
            Local_point_check[i] = 0;
    }
}

double Line_Feature::weight_calculate(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
{
    // !!!!!!!제일 중요한 부분!!!!!!!
    // PRE CONDITION : PT_X, PT_Y, ROBOT_X, ROBOT_Y, VECTOR<VISION_POINT> = VISION_POINT_VECT
    // POST CONDITION : weight
    // PURPOSE : 가중치를 구하는 함수

    // 파티클 위치의 구역 번호 계산
    zone = get_grid_index(PT_X, PT_Y);

    // 실제 관측된 특징점 타입별 개수 산출 (0=L/ㄱ, 1=T, 2=X)
    X_NUM = 0;
    L_NUM = 0;
    T_NUM = 0;
    N_NUM = 0;
    for (int i = 0; i < (int)vect.size(); i++)
    {
        switch (vect[i].FEATURE_TYPE)
        {
        case FieldCoord::X_SHAPE:
            X_NUM++;
            break;
        case FieldCoord::L_SHAPE:
            L_NUM++;
            break;
        case FieldCoord::T_SHAPE:
            T_NUM++;
            break;
        default:
            N_NUM++;
            break;
        }
    }

    // 존별 랜드마크 인덱스 테이블 (compile-time 생성)
    const FieldCoord::ZoneLandmarks &zl = FieldCoord::ZONE_LANDMARKS[zone];

    weight = 0.0;

    for (int i = 0; i < (int)vect.size(); i++)
    {
        ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
        pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;

        min_dis = LARGE_DIST;

        // 관측 방향 (세계 좌표계, POINT_VEC은 이미 yaw 변환됨)
        obs_angle = atan2(vect[i].POINT_VEC_Y, vect[i].POINT_VEC_X);

        // 해당 존의 랜드마크만 순회 (27개 → zone당 평균 1~5개)
        for (int k = 0; k < zl.count; k++)
        {
            int j = zl.indices[k];
            if (FieldCoord::ALL_POINTS[j].type == (FieldCoord::FeatureType)vect[i].FEATURE_TYPE)
            {
                // 파티클 with 후보 랜드마크 방향과 관측 방향 비교 (bearing filter)
                exp_angle = atan2(Local_point_y[j] - PT_Y, Local_point_x[j] - PT_X);
                diff = fabs(obs_angle - exp_angle);
                if (diff > M_PI)
                    diff = 2 * M_PI - diff;
                if (diff > BEARING_MAX_RAD)
                    continue; // 60° 이상 차이나면 skip, 상황에 따라 조정 파라미터

                double dx_m = (Local_point_x[j] - ptx) / FieldCoord::PX_PER_M_X;
                double dy_m = (Local_point_y[j] - pty) / FieldCoord::PX_PER_M_Y;
                dis = sqrt(dx_m * dx_m + dy_m * dy_m);
                if (min_dis > dis)
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
        s = v1 * v1 + v2 * v2;
    } while (s >= 1 || s == 0);
    s = sqrt((-2 * log(s)) / s);
    temp = v1 * s;
    temp = (stdev * temp) + average;
    return temp;
}

void Line_Feature::random_point(ROBOT &robot, int x, int y, float particle_range)
{
    robot.x = x + gaussianRandom(0, particle_range);
    robot.y = y + gaussianRandom(0, particle_range);
}


Rect Line_Feature::get_grid_size(int Num)
{
    // 각 존의 시작점(x, y)과 크기(w, h) — 픽셀 단위
    if (Num == 1)
        return Rect(0, 0, FieldCoord::COL_L, FieldCoord::ROW_T);
    else if (Num == 2)
        return Rect(FieldCoord::COL_L, 0, FieldCoord::COL_R - FieldCoord::COL_L, FieldCoord::ROW_TC);
    else if (Num == 3)
        return Rect(FieldCoord::COL_R, 0, FieldCoord::MAP_W - FieldCoord::COL_R, FieldCoord::ROW_T);
    else if (Num == 4)
        return Rect(0, FieldCoord::ROW_T, FieldCoord::COL_L, FieldCoord::ROW_B - FieldCoord::ROW_T);
    else if (Num == 5)
        return Rect(FieldCoord::COL_L, FieldCoord::ROW_TC, FieldCoord::COL_R - FieldCoord::COL_L, FieldCoord::ROW_BC - FieldCoord::ROW_TC);
    else if (Num == 6)
        return Rect(FieldCoord::COL_R, FieldCoord::ROW_T, FieldCoord::MAP_W - FieldCoord::COL_R, FieldCoord::ROW_B - FieldCoord::ROW_T);
    else if (Num == 7)
        return Rect(0, FieldCoord::ROW_B, FieldCoord::COL_L, FieldCoord::MAP_H - FieldCoord::ROW_B);
    else if (Num == 8)
        return Rect(FieldCoord::COL_L, FieldCoord::ROW_BC, FieldCoord::COL_R - FieldCoord::COL_L, FieldCoord::MAP_H - FieldCoord::ROW_BC);
    else if (Num == 9)
        return Rect(FieldCoord::COL_R, FieldCoord::ROW_B, FieldCoord::MAP_W - FieldCoord::COL_R, FieldCoord::MAP_H - FieldCoord::ROW_B);
    return Rect(0, 0, 0, 0);
}

Mat Line_Feature::get_grid_likelihood(int Num)
{
    Rect bounds(0, 0, FieldCoord::MAP_W, FieldCoord::MAP_H);
    Rect r = get_grid_size(Num);
    Mat roi = Likelihood_mat(r & bounds);
    return roi;
}

int Line_Feature::get_grid_index(int X, int Y)
{
    const bool left = (X >= 0 && X <= FieldCoord::COL_L);
    const bool center = (X > FieldCoord::COL_L && X <= FieldCoord::COL_R);
    const bool right = (X > FieldCoord::COL_R && X <= FieldCoord::MAP_W);

    if (left)
    {
        if (Y >= 0 && Y <= FieldCoord::ROW_T)
            return 1;
        if (Y > FieldCoord::ROW_T && Y <= FieldCoord::ROW_B)
            return 4;
        if (Y > FieldCoord::ROW_B && Y <= FieldCoord::MAP_H)
            return 7;
    }
    else if (center)
    {
        if (Y >= 0 && Y <= FieldCoord::ROW_TC)
            return 2;
        if (Y > FieldCoord::ROW_TC && Y <= FieldCoord::ROW_BC)
            return 5;
        if (Y > FieldCoord::ROW_BC && Y <= FieldCoord::MAP_H)
            return 8;
    }
    else if (right)
    {
        if (Y >= 0 && Y <= FieldCoord::ROW_T)
            return 3;
        if (Y > FieldCoord::ROW_T && Y <= FieldCoord::ROW_B)
            return 6;
        if (Y > FieldCoord::ROW_B && Y <= FieldCoord::MAP_H)
            return 9;
    }
    return 0;
}
// 9그리드 계산

bool sort_return(MEASUREMENT x, MEASUREMENT y)
{
    // PRE CONDITION : MEASUREMENT 구조체 두개
    // POST CONDITION : 두 구조체 중 WEIGHT멤버가 더 큰 쪽이 리턴
    // purpose : particle_weight 벡터 컨테이너에서 sort함수를 사용하기 위한 매개함수
    return x.WEIGHT > y.WEIGHT;
}
