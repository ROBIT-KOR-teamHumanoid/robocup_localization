#ifndef ROBOT_H
#define ROBOT_H

#define DEG2RAD (M_PI / 180)
#include <iostream>
#include <vector>
#include <cmath>

#include "robocup_localization/core/types/pose2d.hpp"
#include "robocup_localization/core/types/trajectory_point.hpp"

using namespace std;

class ROBOT : public robocup_localization::core::Pose2D
{

public:
    using TrajectoryPoint = robocup_localization::core::TrajectoryPoint;

    TrajectoryPoint point;
    vector<TrajectoryPoint> trail;

    // 오도메트리값
    float odom_fx = 0.30;
    float odom_bx = 0.30;

    float odom_ly = 0.30;
    float odom_ry = 0.30;

    // 현재 들어오는 ik파라미터 값
    float now_ik_param_x = 0;
    float now_ik_param_y = 0;

    ROBOT();

    void appendTrail(double Xmoved, double Ymoved);
    void move(double Xmoved, double Ymoved);

    // rot1/trans/rot2 모션 모델 alpha 계수 (튜닝 가능)
    // alpha1: rot 노이즈 ← rot 크기
    // alpha2: rot 노이즈 ← trans 크기
    // alpha3: trans 노이즈 ← trans 크기
    // alpha4: trans 노이즈 ← rot 크기
    // alpha5: yaw 노이즈 ← yaw delta 크기
    //
    // 튜닝 순서
    //   1단계: ALPHA3부터 (위치 노이즈 제일 체감)
    //          로봇 직진 → 파티클 퍼짐이 실제 오차와 비슷하면 OK
    //   2단계: ALPHA1 (방향 노이즈)
    //          제자리 회전 → 랜드마크 매칭 후 수렴 속도 확인
    //   3단계: ALPHA5 (yaw 노이즈)
    //          IMU 드리프트 구간에서 파티클 θ가 랜드마크로 보정되는지 확인
    //   4단계: ALPHA2, ALPHA4는 마지막에 미세 조정
    static constexpr double ALPHA1 = 0.01;
    static constexpr double ALPHA2 = 0.001;
    static constexpr double ALPHA3 = 0.03;
    static constexpr double ALPHA4 = 0.01;
    static constexpr double ALPHA5 = 0.02;

    // 이동/회전 크기에 비례하는 노이즈 표준편차. predict 단계에 넘어간다.
    static double translationStddev(double trans_px, double rot_rad);
    static double directionStddev(double trans_px, double rot_rad);
    static double yawStddev(double yaw_delta_deg);
};

#endif // ROBOT_H
