#pragma once

#include <cmath>
#include <vector>
#include <opencv2/core.hpp>

#include "field_coordinate/field_coordinate.hpp"
#include "robocup_localization/core/mcl/mcl_tune.hpp"
#include "robocup_localization/core/types/vision_point.hpp"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180)
#endif

class PreFilter
{
public:
    PreFilter();

    void setCircle(double yaw, std::vector<VISION_POINT>& vect);

    void checkLocalPoint(int ago_point_cnt, double yaw, std::vector<VISION_POINT>& vect);

    void dropUnmatched(std::vector<VISION_POINT>& vect);

    static bool passBearing(double obs_angle, double exp_angle);

    // 1: y > 0, -1: y < 0, 0: undetermined. Both goalposts sit at y = 0, so either works.
    static int computeYSign(double robot_yaw_deg, const std::vector<double>& goal_post_pan_degs);

    cv::Point CIRCLE_CENTER = cv::Point(0, 0);
    double    CIRCLE_R      = 0;

    int Local_point_x[FieldCoord::FEATURE_POINT_COUNT];
    int Local_point_y[FieldCoord::FEATURE_POINT_COUNT];

    // on_off: landmarks matched by dropUnmatched (UI only).
    // check: landmarks inside the view circle (used for synthetic observations).
    int Local_point_on_off[FieldCoord::FEATURE_POINT_COUNT];
    int Local_point_check[FieldCoord::FEATURE_POINT_COUNT];

    static constexpr double BEARING_MAX_RAD   = M_PI / 9.0;
    static constexpr double COS_DEAD_ZONE     = 0.3;  // |cos(yaw + pan)| below this cannot decide the y sign

    double OBS_FILTER_DIST_M = MclTune::OBS_FILTER_DIST_M;
    double OBS_FILTER_X_FEATURE_DIST_M = MclTune::OBS_FILTER_X_FEATURE_DIST_M;

    static constexpr double CIRCLE_MARGIN_M = MclTune::CIRCLE_MARGIN_M;
    static constexpr int    CIRCLE_MARGIN_PX =
        static_cast<int>(CIRCLE_MARGIN_M * FieldCoord::PX_PER_M_X);

    static constexpr double T_SHAPE_CIRCLE_MARGIN_M = MclTune::T_SHAPE_CIRCLE_MARGIN_M;
    static constexpr int    T_SHAPE_CIRCLE_MARGIN_PX =
        static_cast<int>(T_SHAPE_CIRCLE_MARGIN_M * FieldCoord::PX_PER_M_X);

    static constexpr double LARGE_DIST        = 99999999.0;

private:
    void computeCircle(std::vector<VISION_POINT>& vect, int start, double yaw);

    double X_MAX, X_MIN, Y_MAX, Y_MIN;
    double transformation_x, transformation_y;
    double dis;
};
