#pragma once

#include <cmath>
#include <QGraphicsScene>
#include <QPen>
#include <QBrush>
#include <QPainterPath>
#include <QGraphicsPathItem>

#include "../field_coordinate/field_coordinate.hpp"

namespace technical_challenge
{

static constexpr double APPROACH_RADIUS_M  = 0.25; // 호 반지름
static constexpr double APPROACH_RADIUS_PX = APPROACH_RADIUS_M * FieldCoord::PX_PER_M_X;

// 로봇 정면 기준 반원을 씬에 그림
// cx, cy: 로봇 화면 픽셀 좌표, yaw_deg: 로봇 yaw (도)
inline void draw_approach_circle(QGraphicsScene *scene, int cx, int cy, double yaw_deg)
{
    QPen pen(QColor(0, 200, 255));
    pen.setWidth(2);

    int r = static_cast<int>(APPROACH_RADIUS_PX);

    // 로봇 정면 방향 벡터 (screen: sin(-z), -cos(z))
    // Qt arcTo: 12시 방향이 -90도, 반시계가 양수
    // 정면 각도(Qt 기준) = atan2(cos(z), -sin(z)) * 180/π
    double z_rad = yaw_deg * M_PI / 180.0;
    double fwd_qt_deg = atan2(cos(z_rad), -sin(z_rad)) * 180.0 / M_PI;

    // 정면 중심으로 ±90도 호
    double start_angle = fwd_qt_deg - 90.0;

    QPainterPath path;
    path.moveTo(cx, cy);
    path.arcTo(cx - r, cy - r, r * 2, r * 2, start_angle, 180.0);
    path.closeSubpath();

    scene->addPath(path, pen, QBrush(Qt::NoBrush));
}

// 공이 로봇 중심 25cm 반원(정면) 안에 있는지 확인
// robot_x, robot_y, ball_x, ball_y: 픽셀 좌표, yaw_deg: 로봇 yaw (도)
inline bool is_ball_in_approach_circle(float robot_x, float robot_y,
                                       float ball_x,  float ball_y,
                                       double yaw_deg)
{
    double dx = ball_x - robot_x;
    double dy = ball_y - robot_y;

    if ((dx * dx + dy * dy) > (APPROACH_RADIUS_PX * APPROACH_RADIUS_PX))
        return false;

    // 공 방향이 정면 반원(±90도) 안인지 확인
    double z_rad = yaw_deg * M_PI / 180.0;
    // 로봇 정면 방향 (screen 좌표계)
    double fwd_x = -sin(z_rad);
    double fwd_y = -cos(z_rad);
    // 공 방향과 정면의 내적 > 0 이면 정면 반구
    return (dx * fwd_x + dy * fwd_y) >= 0.0;
}

} // namespace technical_challenge
