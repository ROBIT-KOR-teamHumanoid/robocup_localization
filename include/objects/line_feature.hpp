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

#define DEG2RAD (M_PI / 180)

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
    struct VISION_POINT
    {
        // 비전에서 찾은 포인트 데이터를 저장하는 구조체

        double CONFIDENCE; // 포인트 데이터의 신뢰도
        double DISTANCE;   // 포인트 데이터의 거리

        // 특징점과 로봇 사이의 거리 (픽셀 단위)
        int POINT_VEC_X;
        int POINT_VEC_Y;

        // 로봇의 픽셀 좌표
        int STD_X;
        int STD_Y;

        // 특징점 타입 (0=L/ㄱ, 1=T, 2=X)
        int FEATURE_TYPE;
    };

    void featureCalc(QNode *qnode, double particle_range);
    void set_circle(double yaw, vector<VISION_POINT> &vect);
    void check_local_point(int ago_point_cnt, double yaw, vector<VISION_POINT> &vect);
    double weight_calculate(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect);

    static double gaussianRandom(double average, double stdev);
    static void random_point(ROBOT &robot, int x, int y, float particle_range);

    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect; // VISION_POINT 를 저장할 벡터 컨테이너

    Point CIRCLE_CENTER = Point(0, 0);
    double CIRCLE_R = 0;

    // 특징점의 픽셀 좌표 (field_coordinate.hpp의 ALL_POINTS로 자동 계산)
    int Local_point_x[FieldCoord::FEATURE_POINT_COUNT];
    int Local_point_y[FieldCoord::FEATURE_POINT_COUNT];

    // 특징점 활성화 변수
    int Local_point_on_off[FieldCoord::FEATURE_POINT_COUNT];
    int Local_point_check[FieldCoord::FEATURE_POINT_COUNT];

    // 맵 이미지에 찍힐 특징점 27개의 픽셀 좌표를 미리 계산해서 저장
    Line_Feature()
    {
        for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
        {
            Local_point_x[i] = FieldCoord::toPixelX(FieldCoord::ALL_POINTS[i].x);
            Local_point_y[i] = FieldCoord::toPixelY(FieldCoord::ALL_POINTS[i].y);
            Local_point_on_off[i] = 0;
            Local_point_check[i] = 0;
        }
    }

    // weight_calculate() 튜닝 파라미터
    // weight 공식: weight += (WEIGHT_SCALE / min_dis) * CONFIDENCE * abs(1 - DISTANCE / DIST_NORM)

    // min_dis 초기값. 파티클별로 가장 가까운 랜드마크를 찾기 전 비교 기준값 (사실상 무한대)
    static constexpr double LARGE_DIST = 99999999.0;

    // min_dis 최솟값 클램프 (미터). 파티클 추정 위치와 실제 랜드마크 간 거리의 하한선.
    // 너무 작으면 완벽히 일치하는 파티클 하나가 weight를 독점함.
    // 너무 크면 위치 오차가 커도 동일한 최대 weight → 정밀도 저하.
    // 실제 검출 오차(픽셀 노이즈 + PAN 오차 등)에 맞게 설정. 현재 ~10cm 오차 가정.
    static constexpr double MIN_DIS_CLAMP = 0.1;

    // weight 분자 스케일. 전체 weight 크기를 결정하며 weight_update_threshold와 함께 조정.
    // MIN_DIS_CLAMP 변경 시 max_weight_per_obs = WEIGHT_SCALE / MIN_DIS_CLAMP 가 바뀌므로
    // threshold도 같이 재조정 필요.
    static constexpr double WEIGHT_SCALE = 0.45;

    // 특징점까지 거리(mm) 정규화 분모. 멀리 있는 특징점일수록 weight를 줄이는 계수.
    // abs(1 - DISTANCE / DIST_NORM): DISTANCE=0 → 1.0, DISTANCE=DIST_NORM → 0.0
    // 현재 10000mm(10m) → 실용 범위(0~2m)에서는 패널티 거의 없음 (0.8~1.0)
    static constexpr double DIST_NORM = 10000.0;

    // bearing 필터 임계값 (라디안). 파티클 기준 예상 랜드마크 방향과
    // 실제 관측 방향의 차이가 이 각도를 초과하면 해당 관측을 weight 계산에서 제외.
    // 현재 60° → 정면 ±60° 범위 밖 관측은 무시.
    static constexpr double BEARING_MAX_RAD = M_PI / 3.0;

    // 한 번의 MATCHING에서 robot0 위치가 이동할 수 있는 최대 거리 (미터).
    // 이 값을 초과하는 점프는 오매칭으로 보고 위치 업데이트를 거부함.
    static constexpr double MAX_JUMP_M = 0.7;
    static constexpr double MAX_JUMP_PX = MAX_JUMP_M * FieldCoord::PX_PER_M_X;

    // pre-filter 임계값 (미터). featureCalc 시작 시 각 관측에 대해
    // (robot0 현재 위치 + POINT_VEC)로 추정한 월드 좌표와 가장 가까운 실제 랜드마크 거리를 계산.
    // 이 거리가 임계값 초과이면 해당 관측을 버림 → 명백히 틀린 관측 제거.
    // 너무 작으면 robot0가 약간 틀려도 정상 관측이 버려짐.
    // 너무 크면 엉뚱한 랜드마크와 매칭된 관측도 통과 → weight 오염.
    static constexpr double OBS_FILTER_DIST_M = 1.5;

    vector<MEASUREMENT> particle_weight;

private:
    Rect get_grid_size(int Num);
    Mat get_grid_likelihood(int Num);
    int get_grid_index(int X, int Y);
    void compute_circle(vector<VISION_POINT> &vect, int start, double yaw);

    Mat Likelihood_mat;

    // set_circle / check_local_point 작업 변수
    double X_MAX, X_MIN, Y_MAX, Y_MIN;
    double transformation_x, transformation_y;
    double dis;

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
    int zone;
    int X_NUM, L_NUM, T_NUM, N_NUM;
    double weight;
    double min_dis;
    int ptx, pty;
    double obs_angle;
    double exp_angle, diff;
};

#endif // LINE_FEATURE_H
