#pragma once

#include <cmath>
#include <vector>
#include <opencv2/core.hpp>

#include "../field_coordinate/field_coordinate.hpp"
#include "../objects/vision_point.hpp"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180)
#endif

class Filter
{
public:
    Filter();

    // 관측 범위 원 설정 (featureCalc 시작 시)
    void setCircle(double yaw, std::vector<VISION_POINT>& vect);

    // 원 범위 내 활성 랜드마크 결정 (featureCalc 마지막 단계)
    void checkLocalPoint(int ago_point_cnt, double yaw, std::vector<VISION_POINT>& vect);

    // 관측 pre-filter: 가장 가까운 랜드마크와의 거리가 임계값 초과이면 제거
    void prefilter(std::vector<VISION_POINT>& vect);

    // bearing 필터: 관측 방향과 예상 방향 차이가 임계값 이내이면 true
    static bool passBearing(double obs_angle, double exp_angle);

    // 골대 pan 각도 + 현재 yaw로 로봇 y부호 결정 (1: y>0, -1: y<0, 0: 미결정)
    // 두 골대 모두 y=0이므로 어느 골대든 동일하게 적용됨
    static int computeYSign(double robot_yaw_deg, const std::vector<double>& goal_post_pan_degs);

    // 원 결과 (double 픽셀)
    cv::Point2d CIRCLE_CENTER = cv::Point2d(0, 0);
    double      CIRCLE_R      = 0;

    // 랜드마크 픽셀 좌표 (ALL_POINTS 기반 사전 계산, double 픽셀)
    double Local_point_x[FieldCoord::FEATURE_POINT_COUNT];
    double Local_point_y[FieldCoord::FEATURE_POINT_COUNT];

    // 랜드마크 활성화 플래그
    int Local_point_on_off[FieldCoord::FEATURE_POINT_COUNT];
    int Local_point_check[FieldCoord::FEATURE_POINT_COUNT];

    // 필터 파라미터
    // bearing 임계값 (라디안). 예상 방향과 관측 방향 차이가 이 값 초과이면 랜드마크 skip.
    static constexpr double BEARING_MAX_RAD   = M_PI / 9.0;  // 20°
    static constexpr double COS_DEAD_ZONE     = 0.3;          // |cos(yaw-pan)| < 0.3 이면 y부호 판별 불가

    // pre-filter 거리 임계값 (미터). 가장 가까운 랜드마크가 이 값 초과이면 관측 제거.
    double OBS_FILTER_DIST_M = 1.7;
    double OBS_FILTER_X_FEATURE_DIST_M = 2.0; // X-shape 관측 시 pre-filter 거리 임계값 (미터)

    static constexpr double LARGE_DIST        = 99999999.0;

private:
    void computeCircle(std::vector<VISION_POINT>& vect, int start, double yaw);

    // computeCircle 작업 변수
    double X_MAX, X_MIN, Y_MAX, Y_MIN;
    double transformation_x, transformation_y;
    double dis;
};
