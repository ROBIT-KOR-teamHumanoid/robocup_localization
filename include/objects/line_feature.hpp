#ifndef LINE_FEATURE_H
#define LINE_FEATURE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>

#include "../field_coordinate/field_coordinate.hpp"
#include "../objects/robot.hpp"
#include "../objects/vision_point.hpp"
#include "../robocup_localization/filter.hpp"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180)
#endif

using namespace std;
using namespace cv;

struct MEASUREMENT
{
    int NUM;
    double WEIGHT;
};

bool sort_return(MEASUREMENT x, MEASUREMENT y);

class QNode;

class Line_Feature
{
public:
    void featureCalc(QNode *qnode, double particle_range);
    double weight_calculate(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect);

    static double gaussianRandom(double average, double stdev);
    static void random_point(ROBOT &robot, int x, int y, float particle_range);

    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect;

    Filter filter;

    Line_Feature() {}

    // weight_calculate() 튜닝 파라미터
    // weight 공식: weight += (WEIGHT_SCALE / min_dis) * CONFIDENCE * abs(1 - DISTANCE / DIST_NORM)

    // min_dis 초기값. 파티클별로 가장 가까운 랜드마크를 찾기 전 비교 기준값 (사실상 무한대)
    static constexpr double LARGE_DIST = 99999999.0;

    // min_dis 최솟값 클램프 (미터). 파티클 추정 위치와 실제 랜드마크 간 거리의 하한선.
    // 너무 작으면 완벽히 일치하는 파티클 하나가 weight를 독점함.
    // 너무 크면 위치 오차가 커도 동일한 최대 weight → 정밀도 저하.
    static constexpr double MIN_DIS_CLAMP = 0.1;

    // weight 분자 스케일. 전체 weight 크기를 결정하며 weight_update_threshold와 함께 조정.
    static constexpr double WEIGHT_SCALE = 0.6;

    // 특징점까지 거리(mm) 정규화 분모. 멀리 있는 특징점일수록 weight를 줄이는 계수.
    static constexpr double DIST_NORM = 15000.0;

    // L_SHAPE 거리 매칭 게이트 (미터). 관측 거리와 실제 거리 차이가 이 값 초과이면 랜드마크 skip.
    static constexpr double L_DIST_GATE_M = 0.5;

    // 한 번의 MATCHING에서 robot0 위치가 이동할 수 있는 최대 거리 (미터).
    static constexpr double MAX_JUMP_M  = 0.7;
    static constexpr double MAX_JUMP_PX = MAX_JUMP_M * FieldCoord::PX_PER_M_X;

    // top-N 파티클 위치 분산 허용 한계 (픽셀). 이보다 퍼져있으면 업데이트 reject.
    static constexpr double MAX_SPREAD_M  = 0.5;
    static constexpr double MAX_SPREAD_PX = MAX_SPREAD_M * FieldCoord::PX_PER_M_X;

    vector<MEASUREMENT> particle_weight;

private:
    // featureCalc 작업 변수
    int obs_size;
    MEASUREMENT fc_meas;
    double total_weight;
    int top_n;
    double sum_w, wx, wy;
    vector<double> cdf;
    double fc_step, fc_r;
    vector<float> new_x, new_y;
    int fc_j;
    int synthetic_cnt;
    double resample_noise;

    // weight_calculate 작업 변수
    double weight;
    double min_dis;
    int ptx, pty;
    double obs_angle;
    double exp_angle;

    // 공 필드 좌표 (featureCalc에서 세팅, weight_calculate에서 사용)
    double ball_field_px_x;
    double ball_field_px_y;
    bool   ball_visible;
};

#endif // LINE_FEATURE_H
