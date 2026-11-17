#pragma once

#include <cmath>
#include "field_coordinate/field_coordinate.hpp"

struct VISION_POINT
{
    double CONFIDENCE;
    double DISTANCE;

    // World-frame pixels, rotated by robot0.yaw_deg.
    int POINT_VEC_X;
    int POINT_VEC_Y;

    // Robot frame mm before yaw rotation; re-rotated per particle.
    double RAW_X_MM;
    double RAW_Y_MM;

    int STD_X;
    int STD_Y;

    int FEATURE_TYPE;  // 0 = L, 1 = T, 2 = X

    double BALL_RELATIVE_ANGLE;
    double BALL_FEATURE_DISTANCE;
};

class VisionPointUtils
{
public:
    static int worldX(const VISION_POINT& vp) { return vp.STD_X + vp.POINT_VEC_X; }
    static int worldY(const VISION_POINT& vp) { return vp.STD_Y + vp.POINT_VEC_Y; }

    static double bearing(const VISION_POINT& vp) { return atan2(vp.POINT_VEC_Y, vp.POINT_VEC_X); }

    static VISION_POINT makeSynthetic(int lx, int ly, int robot_x, int robot_y, int type, double robot_yaw_deg)
    {
        VISION_POINT vp;
        vp.CONFIDENCE            = 0.1;
        vp.DISTANCE              = 1;
        vp.POINT_VEC_X           = lx - robot_x;
        vp.POINT_VEC_Y           = ly - robot_y;
        vp.STD_X                 = robot_x;
        vp.STD_Y                 = robot_y;
        vp.FEATURE_TYPE          = type;
        vp.BALL_RELATIVE_ANGLE   = 0.0;
        vp.BALL_FEATURE_DISTANCE = 0.0;

        double pvx_m = vp.POINT_VEC_X / FieldCoord::PX_PER_M_X;
        double pvy_m = vp.POINT_VEC_Y / FieldCoord::PX_PER_M_Y;
        double c = std::cos(robot_yaw_deg * M_PI / 180.0);
        double s = std::sin(robot_yaw_deg * M_PI / 180.0);
        vp.RAW_X_MM = (-pvx_m * c + pvy_m * s) * 1000.0;
        vp.RAW_Y_MM = (-pvx_m * s - pvy_m * c) * 1000.0;

        return vp;
    }
};
