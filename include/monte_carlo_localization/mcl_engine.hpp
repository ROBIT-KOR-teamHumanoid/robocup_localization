#ifndef MCL_ENGINE_H
#define MCL_ENGINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <mutex>

#include "field_profile/field_coordinate.hpp"
#include "objects_model/particle.hpp"
#include "objects_model/self_robot.hpp"
#include "objects_model/teammate.hpp"
#include "objects_model/vision_point.hpp"
#include "monte_carlo_localization/filter.hpp"

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

class MclNode;

class MclEngine
{
public:
    // MCL 한 사이클: 가중치 계산 → pose 보정 → 리샘플 → synthetic 재주입.
    // 락(vision → pt → robot0)은 이 함수가 직접 잡는다.
    void featureCalc(MclNode &mcl);
    double weight_calculate(double PT_X, double PT_Y, float PT_Z, double NOW_X, double NOW_Y, vector<VISION_POINT> &vect);

    static double gaussianRandom(double average, double stdev);
    static void random_point(Particle &p, int x, int y, float particle_range, float theta);

    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect;

    Filter filter;

    // 골대 관측으로 결정된 로봇 y부호 (1: y>0, -1: y<0, 0: 미결정)
    int robot_y_sign = 0;

    MclEngine() {}

    // weight_calculate() 튜닝 파라미터
    // weight 공식: Mahalanobis distance 기반
    //   D^2 = dn^2/sigma_n^2 + dp^2/sigma_p^2
    //   weight += CONFIDENCE * exp(-0.5 * D^2)
    //   dn: 파티클→랜드마크 방향 오차(normal), dp: 수직 방향 오차(perp)
    //   sigma = INV_*_VAR * dist_m + 0.05 (거리에 비례해서 커짐)

    // min_dis 초기값. 파티클별로 가장 가까운 랜드마크를 찾기 전 비교 기준값 (사실상 무한대)
    static constexpr double LARGE_DIST = 99999999.0;

    // 거리(미터) 임계값 — L_DIST_GATE_M / MIN_JUMP / MAX_JUMP / MAX_SPREAD —
    // 은 필드 크기에 따라 달라지므로 filter.hpp 의 MclTune 이 소유한다.
    // 아래 계수들은 스케일 불변이라 여기 남는다.

    // normal 방향 분산 계수 (거리 비례). 클수록 normal 오차에 관대해짐.
    // sigma = 계수 * dist_m + 0.05 이라 이미 관측 거리에 비례 → 스케일 불변.
    static constexpr double INV_NORM_VAR = 0.1;

    // perp 방향 분산 계수. INV_NORM_VAR보다 크게 설정 → 수직 오차에 더 관대.
    static constexpr double INV_PERP_VAR = 0.2;

    // resampling 시 파티클 θ 초기 분산 (deg). 너무 작으면 theta 다양성 소실.
    // 각도라 필드 크기와 무관.
    static constexpr double THETA_INIT_SPREAD_DEG = 0;


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

#endif // MCL_ENGINE_H
