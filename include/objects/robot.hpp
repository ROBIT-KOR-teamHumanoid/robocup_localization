#ifndef ROBOT_H
#define ROBOT_H

#define DEG2RAD (M_PI / 180)
#include <ctime>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class ROBOT
{

public:
    struct POINT
    {
        // 로봇이 지나간 경로를 그리기 위한 구조체
        unsigned int TIME;
        int X;
        int Y;
    };
    POINT point;
    vector<POINT> TIME_STAMP; // TIME_STAMP를 저장할 벡터 컨테이너

    int state = 0;

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    float b = 0.0;

    // 오도메트리값
    float odom_fx = 0.30;
    float odom_bx = 0.30;

    float odom_ly = 0.30;
    float odom_ry = 0.30;

    // 현재 들어오는 ik파라미터 값
    float now_ik_param_x = 0;
    float now_ik_param_y = 0;

    double weight; // 파티클이 가지는 가중치값, 파티클에만 사용되는 값이고 로봇에는 사용되지 않음

    ROBOT();
    ROBOT(float x, float y);

    void create_time_stamp(double Xmoved, double Ymoved);
    void move(double Xmoved, double Ymoved);
};

#endif // ROBOT_H
