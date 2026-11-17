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

    // Ik: IK command scaled by the odom coefficients below.
    // Metric: real motion in cm (learned odometry, sim GT); coefficients are ignored.
    enum class OdometryInput
    {
        Ik,
        Metric,
    };
    OdometryInput odom_input = OdometryInput::Ik;

    // robocup_learned_odometry odom_node and k1_mujoco_bridge both send cm.
    static constexpr double METRIC_UNITS_PER_METER = 100.0;

    // Ik only: map-x pixels per input unit, per direction.
    float odom_fx = 0.30;
    float odom_bx = 0.30;

    float odom_ly = 0.30;
    float odom_ry = 0.30;

    float now_ik_param_x = 0;
    float now_ik_param_y = 0;

    ROBOT();

    void appendTrail(double Xmoved, double Ymoved);
    void move(double Xmoved, double Ymoved);

    // alpha1..4: rot<-rot, rot<-trans, trans<-trans, trans<-rot.
    // Tune ALPHA3 first, then ALPHA1; ALPHA2/ALPHA4 last.
    static constexpr double ALPHA1 = 0.01;
    static constexpr double ALPHA2 = 0.001;
    static constexpr double ALPHA3 = 0.03;
    static constexpr double ALPHA4 = 0.01;

    static double translationStddev(double trans_px, double rot_rad);
    static double directionStddev(double trans_px, double rot_rad);
};

#endif
