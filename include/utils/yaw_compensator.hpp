#pragma once

#include <vector>

// 골대 관측으로 yaw 드리프트 보정
// ============================================================
// IMU yaw 는 시간이 지나면 드리프트한다. 골대는 필드에서 위치가 확실한
// 랜드마크라, "현재 추정 위치에서 저 골대를 보려면 yaw 가 얼마여야 하는가"를
// 역산해 관측된 pan 각과 맞춰보면 드리프트를 잡을 수 있다.
//
// 골대가 둘이고 관측 pan 도 여러 개라 후보가 여러 개 나온다. 그중 현재 yaw 와
// 가장 가까운 것을 고른다 — 가장 보수적인 해석이다.
//
// computeYSign 도 여기 있다. 같은 골대 pan 입력으로 pose 의 다른 모호성
// (y 부호)을 푸는 짝이기 때문이다.
namespace YawCompensator
{

// |cos(yaw+pan)| 이 이 값보다 작으면 (yaw+pan ≈ ±90도) y 부호를 판별할 수 없다.
constexpr double COS_DEAD_ZONE = 0.3;

/// 골대 pan 각도 + 현재 yaw 로 로봇의 y 부호를 결정한다.
///   1: y>0 쪽, -1: y<0 쪽, 0: 미결정
/// 두 골대 모두 y=0 이라 어느 골대를 봤든 동일하게 적용된다.
/// 관측이 여러 개면 다수결로 정하고, 동수면 0(미결정)을 준다.
int computeYSign(double robot_yaw_deg,
                 const std::vector<double> &goal_post_pan_degs);


struct Result
{
  bool valid = false;             // 골대 관측이 하나도 없으면 false
  double corrected_yaw_deg = 0.0; // 골대 기준으로 역산한 yaw
  double drift_deg = 0.0;         // 현재 yaw 와의 차이 (0~180)
};

/// 로봇 위치(미터)와 현재 yaw, 관측된 골대 pan 각들로 보정 yaw 를 추정한다.
/// 순수 계산 — 상태를 만지지 않으므로 호출자가 락 없이 부를 수 있다.
Result estimate(double robot_x_m, double robot_y_m, double current_yaw_deg,
                const std::vector<double> &goal_post_pan_degs);

}  // namespace YawCompensator
