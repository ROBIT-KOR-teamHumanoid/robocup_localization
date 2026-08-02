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
#include "../robot_odom_model/robot_odom_model.hpp"
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
    double weight_calculate(double PT_X, double PT_Y, float PT_Z, double NOW_X, double NOW_Y, vector<VISION_POINT> &vect);

    static double gaussianRandom(double average, double stdev);
    static void random_point(ROBOT &robot, int x, int y, float particle_range, float theta);

    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect;

    Filter filter;

    // 골대 관측으로 결정된 로봇 y부호 (1: y>0, -1: y<0, 0: 미결정)
    int robot_y_sign = 0;

    Line_Feature() {}

    // weight_calculate() 튜닝 파라미터
    // weight 공식: Mahalanobis distance 기반
    //   D^2 = dn^2/sigma_n^2 + dp^2/sigma_p^2
    //   weight += CONFIDENCE * exp(-0.5 * D^2)
    //   dn: 파티클→랜드마크 방향 오차(normal), dp: 수직 방향 오차(perp)
    //   sigma = INV_*_VAR * dist_m + 0.05 (거리에 비례해서 커짐)

    // min_dis 초기값. 파티클별로 가장 가까운 랜드마크를 찾기 전 비교 기준값 (사실상 무한대)
    static constexpr double LARGE_DIST = 99999999.0;

    // 아래 거리(미터) 임계값들은 S-Field 기준 값에 FieldCoord::FIELD_SCALE 을
    // 곱해 현재 필드 크기에 맞춘다. 기준값만 보고 튜닝하면 되고, 필드를 바꾸면
    // robot_config.h 의 define 만 바꾸면 자동으로 따라간다.

    // normal 방향 분산 계수 (거리 비례). 클수록 normal 오차에 관대해짐.
    // sigma = 계수 * dist_m + 0.05 이라 이미 관측 거리에 비례 → 스케일 불변.
    static constexpr double INV_NORM_VAR = 0.1;

    // perp 방향 분산 계수. INV_NORM_VAR보다 크게 설정 → 수직 오차에 더 관대.
    static constexpr double INV_PERP_VAR = 0.2;

    // resampling 시 파티클 θ 초기 분산 (deg). 너무 작으면 theta 다양성 소실.
    // 각도라 필드 크기와 무관.
    static constexpr double THETA_INIT_SPREAD_DEG = 0;

    // L_SHAPE 거리 매칭 게이트 (미터). 관측 거리와 실제 거리 차이가 이 값 초과이면 랜드마크 skip.
    // 거리 추정 오차가 관측 거리에 비례하므로 필드 크기만큼 넓힌다.
    static constexpr double L_DIST_GATE_REF_M = 0.5;
    static constexpr double L_DIST_GATE_M = L_DIST_GATE_REF_M * FieldCoord::FIELD_SCALE;

    // 한 번의 MATCHING에서 robot0 위치가 이동할 수 있는 최소/최대 거리 (미터).
    // MIN_JUMP_PX 미만이면 진동 억제를 위해 업데이트 skip.
    // 진동 폭도 보정량도 관측 거리를 따라 커지므로 둘 다 스케일한다.
    static constexpr double MIN_JUMP_REF_M = 0.07;
    static constexpr double MIN_JUMP_M  = MIN_JUMP_REF_M * FieldCoord::FIELD_SCALE;
    static constexpr double MIN_JUMP_PX = MIN_JUMP_M * FieldCoord::PX_PER_M;
    static constexpr double MAX_JUMP_REF_M = 1.0;
    static constexpr double MAX_JUMP_M  = MAX_JUMP_REF_M * FieldCoord::FIELD_SCALE;
    static constexpr double MAX_JUMP_PX = MAX_JUMP_M * FieldCoord::PX_PER_M;

    // top-N 파티클 위치 분산 허용 한계 (픽셀). 이보다 퍼져있으면 업데이트 reject.
    static constexpr double MAX_SPREAD_REF_M = 0.5;
    static constexpr double MAX_SPREAD_M  = MAX_SPREAD_REF_M * FieldCoord::FIELD_SCALE;
    static constexpr double MAX_SPREAD_PX = MAX_SPREAD_M * FieldCoord::PX_PER_M;

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
    int    best_j;
    double ptx, pty;
    double obs_angle;
    double exp_angle;

    // 공 필드 좌표 (featureCalc에서 세팅, weight_calculate에서 사용)
    double ball_field_px_x;
    double ball_field_px_y;
    bool   ball_visible;
};

#endif // LINE_FEATURE_H
