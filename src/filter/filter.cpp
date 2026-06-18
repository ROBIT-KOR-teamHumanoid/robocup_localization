#include "../../include/robocup_localization/filter.hpp"

#include <cmath>

Filter::Filter()
{
    for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
    {
        Local_point_x[i]      = FieldCoord::toPixelX(FieldCoord::ALL_POINTS[i].x);
        Local_point_y[i]      = FieldCoord::toPixelY(FieldCoord::ALL_POINTS[i].y);
        Local_point_on_off[i] = 0;
        Local_point_check[i]  = 0;
    }
}

void Filter::computeCircle(std::vector<VISION_POINT>& vect, int start, double yaw)
{
    
    // POST CONDITION : CIRCLE_CENTER, CIRCLE_R
    // PURPOSE : 감지된 특징점들의 외접원 계산 (setCircle / checkLocalPoint 공통 로직)

    X_MAX = -9999;
    X_MIN =  9999;
    Y_MAX = -9999;
    Y_MIN =  9999;

    for (int i = start; i < (int)vect.size(); i++)
    {
        transformation_x = vect[i].POINT_VEC_X * cos(-yaw * DEG2RAD) - vect[i].POINT_VEC_Y * sin(-yaw * DEG2RAD);
        transformation_y = vect[i].POINT_VEC_X * sin(-yaw * DEG2RAD) + vect[i].POINT_VEC_Y * cos(-yaw * DEG2RAD);
        if (transformation_x > X_MAX) X_MAX = transformation_x;
        if (transformation_x < X_MIN) X_MIN = transformation_x;
        if (transformation_y > Y_MAX) Y_MAX = transformation_y;
        if (transformation_y < Y_MIN) Y_MIN = transformation_y;
    }

    CIRCLE_CENTER = cv::Point(
        (int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2) * cos(yaw * DEG2RAD) - ((Y_MAX + Y_MIN) / 2) * sin(yaw * DEG2RAD)),
        (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2) * sin(yaw * DEG2RAD) + ((Y_MAX + Y_MIN) / 2) * cos(yaw * DEG2RAD)));

    CIRCLE_R = 0;
    for (int i = start; i < (int)vect.size(); i++)
    {
        dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) +
                   pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
        if (CIRCLE_R < dis)
            CIRCLE_R = dis;
    }
    CIRCLE_R += FieldCoord::CIRCLE_MARGIN_PX;
}

void Filter::setCircle(double yaw, std::vector<VISION_POINT>& vect)
{
    // PRE CONDITION : YAW, VECTOR<VISION_POINT>
    // POST CONDITION : CIRCLE_CENTER, CIRCLE_R
    // PURPOSE : 로컬의 특징점 범위를 설정

    if (vect.empty())
        return;
    computeCircle(vect, 0, yaw);

    // T자 특징점은 분포가 드물어 서클 범위를 추가로 확장
    bool has_t_shape = false;
    for (const auto& vp : vect)
    {
        if (vp.FEATURE_TYPE == (int)FieldCoord::T_SHAPE)
        {
            has_t_shape = true;
            break;
        }
    }
    if (has_t_shape)
        CIRCLE_R += FieldCoord::T_SHAPE_CIRCLE_MARGIN_PX;
}

void Filter::checkLocalPoint(int ago_point_cnt, double yaw, std::vector<VISION_POINT>& vect)
{
    // PRE CONDITION : ago_point_cnt, yaw, vect<VISION_POINT>
    // POST CONDITION : Local_point_check
    // PURPOSE : 로봇의 yaw값을 통해 파티클 필터 계산에 사용될 특징점 활성화

    if (vect.empty() || ago_point_cnt >= (int)vect.size())
        return;

    computeCircle(vect, ago_point_cnt, yaw);

    bool has_t_shape = false;
    for (int k = ago_point_cnt; k < (int)vect.size(); k++)
    {
        if (vect[k].FEATURE_TYPE == (int)FieldCoord::T_SHAPE)
        {
            has_t_shape = true;
            break;
        }
    }
    if (has_t_shape)
        CIRCLE_R += FieldCoord::T_SHAPE_CIRCLE_MARGIN_PX;

    for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
    {
        if (pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y[i], 2))
            Local_point_check[i] = 1;
        else
            Local_point_check[i] = 0;
    }
}

void Filter::prefilter(std::vector<VISION_POINT>& vect)
{
 
    // PURPOSE : robot0 기준 estimated world 좌표와 nearest landmark 거리가 
    //           OBS_FILTER_DIST_M 초과인 관측 제거

    auto it = vect.begin();
    while (it != vect.end())
    {
        int wx = VisionPointUtils::worldX(*it);
        int wy = VisionPointUtils::worldY(*it);
        double nearest = LARGE_DIST;

        for (int j = 0; j < FieldCoord::FEATURE_POINT_COUNT; j++)
        {
            if (!FieldCoord::ALL_POINTS[j].active)
                continue;
            if (FieldCoord::ALL_POINTS[j].type == (FieldCoord::FeatureType)it->FEATURE_TYPE)
            {
                double dx_m = (Local_point_x[j] - wx) / FieldCoord::PX_PER_M_X;
                double dy_m = (Local_point_y[j] - wy) / FieldCoord::PX_PER_M_Y;
                double d    = sqrt(dx_m * dx_m + dy_m * dy_m);
                if (d < nearest)
                    nearest = d;
            }
        }

        if (nearest > OBS_FILTER_DIST_M)
            it = vect.erase(it);
        else
            ++it;
    }
}

bool Filter::passBearing(double obs_angle, double exp_angle)
{
    // PRE CONDITION : obs_angle (관측 방향), exp_angle (예상 방향) - 라디안
    // POST CONDITION : 두 방향 차이가 BEARING_MAX_RAD 이내이면 true
    // PURPOSE : bearing 필터 통과 여부 판단

    double diff = fabs(obs_angle - exp_angle);
    if (diff > M_PI)
        diff = 2 * M_PI - diff;

    if (diff <= BEARING_MAX_RAD)
        return true;
    else
        return false;
}
