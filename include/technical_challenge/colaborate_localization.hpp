#include <cmath>
#include "../field_coordinate/field_coordinate.hpp"

namespace colaborate_localization
{

// 비전에서 관측한 상대 로봇 벡터(mm)를 절대 필드 좌표(m)로 변환
// robot_vec_x, robot_vec_y : 비전 관측 로봇 상대 벡터 (mm)
// robot0_x, robot0_y       : 자신의 현재 픽셀 좌표
// robot0_z                 : 자신의 현재 yaw (도)
// out_field_x, out_field_y : 출력 절대 필드 좌표 (m)
inline void computeBlindRobotCoord(
    float robot_vec_x, float robot_vec_y,
    float robot0_x, float robot0_y, float robot0_z,
    double& out_field_x, double& out_field_y)
{
    double dx = (-1.0) * (robot_vec_x / 1000.0 * FieldCoord::PX_PER_M_X * cos(robot0_z * M_PI / 180.0)
                        + robot_vec_y / 1000.0 * FieldCoord::PX_PER_M_X * sin(robot0_z * M_PI / 180.0));
    double dy = (robot_vec_x / 1000.0 * FieldCoord::PX_PER_M_Y * sin(robot0_z * M_PI / 180.0)
               - robot_vec_y / 1000.0 * FieldCoord::PX_PER_M_Y * cos(robot0_z * M_PI / 180.0));

    out_field_x = robot0_x + dx;
    out_field_y = robot0_y + dy;
}

}
