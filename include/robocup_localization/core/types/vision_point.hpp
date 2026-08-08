#pragma once

#include <cmath>
#include "field_coordinate/field_coordinate.hpp"

struct VISION_POINT
{
    double CONFIDENCE;          // 포인트 데이터의 신뢰도
    double DISTANCE;            // 포인트 데이터의 거리

    // 특징점과 로봇 사이의 거리 (픽셀 단위, robot0.yaw_deg 기준 world-frame)
    int POINT_VEC_X;
    int POINT_VEC_Y;

    // 특징점 원시 로봇 좌표계 거리 (mm, yaw 변환 전)
    // per-particle theta 변환에 사용
    double RAW_X_MM;
    double RAW_Y_MM;

    // 로봇의 픽셀 좌표
    int STD_X;
    int STD_Y;

    // 특징점 타입 (0=L/ㄱ, 1=T, 2=X)
    int FEATURE_TYPE;

    // 이미지 내 특징점→공 방향 각도 (rad), 픽셀 거리
    double BALL_RELATIVE_ANGLE;
    double BALL_FEATURE_DISTANCE;
};

class VisionPointUtils
{
public:
    // 관측점의 추정 필드 좌표 (로봇 위치 + 관측 벡터)
    static int worldX(const VISION_POINT& vp) { return vp.STD_X + vp.POINT_VEC_X; }
    static int worldY(const VISION_POINT& vp) { return vp.STD_Y + vp.POINT_VEC_Y; }

    // 관측 방향 (라디안, yaw 변환 완료된 POINT_VEC 기준)
    static double bearing(const VISION_POINT& vp) { return atan2(vp.POINT_VEC_Y, vp.POINT_VEC_X); }

    // synthetic 관측 생성 (checkLocalPoint 후 다음 사이클용)
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

        // RAW_X_MM/Y: POINT_VEC를 robot yaw로 역변환 (world-frame pixel → robot-frame mm)
        double pvx_m = vp.POINT_VEC_X / FieldCoord::PX_PER_M_X;
        double pvy_m = vp.POINT_VEC_Y / FieldCoord::PX_PER_M_Y;
        double c = std::cos(robot_yaw_deg * M_PI / 180.0);
        double s = std::sin(robot_yaw_deg * M_PI / 180.0);
        vp.RAW_X_MM = (-pvx_m * c + pvy_m * s) * 1000.0;
        vp.RAW_Y_MM = (-pvx_m * s - pvy_m * c) * 1000.0;

        return vp;
    }
};
