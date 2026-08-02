#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "field_profile/field_coordinate.hpp"
#include "objects_model/vision_point.hpp"
#include "monte_carlo_localization/mcl_tune.hpp"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180)
#endif

class Filter
{
public:
    Filter();

    // 랜드마크 픽셀 좌표를 현재 FieldCoord 값으로 재계산.
    // Filter 는 QNode 멤버라 생성자가 파라미터 로드보다 **먼저** 돈다.
    // 따라서 FieldCoord::init() 직후 반드시 한 번 호출해야 한다.
    void rebuild();

    // 관측 범위 원 설정 (featureCalc 시작 시)
    void setCircle(double yaw, std::vector<VISION_POINT>& vect);

    // 원 범위 내 활성 랜드마크 결정 (featureCalc 마지막 단계)
    void checkLocalPoint(int ago_point_cnt, double yaw, std::vector<VISION_POINT>& vect);

    // 관측 pre-filter: 가장 가까운 랜드마크와의 거리가 임계값 초과이면 제거
    void prefilter(std::vector<VISION_POINT>& vect);

    // bearing 필터: 관측 방향과 예상 방향 차이가 임계값 이내이면 true
    static bool passBearing(double obs_angle, double exp_angle);


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
    // 각도 게이트라 필드 크기와 무관 → 스케일하지 않는다.
    static constexpr double BEARING_MAX_RAD   = M_PI / 9.0;  // 20°

    // pre-filter 거리 임계값은 MclTune 으로 옮겼다.
    // 단, "adult 는 prefilter 를 끈다"(qnode.cpp)는 인스턴스 단위 결정이라
    // 여기서 오버라이드를 들고 있는다. 음수면 MclTune 값을 그대로 쓴다.
    double obs_filter_override_m = -1.0;

    double obsFilterDistM() const
    {
        return obs_filter_override_m >= 0.0 ? obs_filter_override_m
                                            : MclTune::OBS_FILTER_DIST_M;
    }
    double obsFilterXFeatureDistM() const
    {
        return obs_filter_override_m >= 0.0 ? obs_filter_override_m
                                            : MclTune::OBS_FILTER_X_FEATURE_DIST_M;
    }

    static constexpr double LARGE_DIST        = 99999999.0;

private:
    void computeCircle(std::vector<VISION_POINT>& vect, int start, double yaw);

    // computeCircle 작업 변수
    double X_MAX, X_MIN, Y_MAX, Y_MIN;
    double transformation_x, transformation_y;
    double dis;
};
