#pragma once

#include <vector>

#include "objects_model/particle.hpp"
#include "objects_model/self_robot.hpp"
#include "objects_model/teammate.hpp"

// 오도메트리 모션 모델 (rot1 / trans / rot2)
// ============================================================
// IkCoordMsg 한 프레임을 받아 robot0 을 이동시키고, 그 델타로 파티클을 전파한다.
// rot2 는 yaw 를 IMU 에서 별도 처리하므로 0 으로 가정한다.
//
// 락은 호출자(QNode)가 잡는다 — applyOdometry 는 robot0, propagate 는 pt 를
// 만지므로 표준 순서(pt -> robot0)를 지키려면 둘을 분리해서 호출해야 한다.
//
//   MotionModel::Delta d;
//   { lock robot0;  d = motion.applyOdometry(robot0, msg->x, msg->y, foot_align); }
//   { lock pt;      motion.propagate(pt, particle_num, d, d_yaw); }
class MotionModel
{
public:
  struct Config
  {
    bool noise_enable = true;
    // FOOT_ALIGN 단계에서는 실제 이동이 명령보다 작아 델타를 축소한다.
    double foot_align_odom_scale = 0.3;

    // ik 명령 단위 → 맵 픽셀 배율. 전/후진과 좌/우 보폭이 달라 방향별로 둔다.
    // GUI 슬라이더로 맞추고 PARAM 파일에 저장된다 (gui/odom_param_store).
    // 예전엔 ROBOT 멤버였는데, 파티클 8000개가 쓰지도 않는 사본을 각자
    // 들고 다녀서 여기로 옮겼다 — 이건 pose 가 아니라 모델 캘리브레이션이다.
    double odom_scale_forward = 0.30;
    double odom_scale_backward = 0.30;
    double odom_scale_left = 0.30;
    double odom_scale_right = 0.30;
  };
  Config config;

  // 한 스텝의 이동량. applyOdometry 가 만들고 propagate 가 소비한다.
  struct Delta
  {
    double dx = 0.0;     // world-frame 픽셀 델타
    double dy = 0.0;
    double trans = 0.0;  // world-frame 이동 거리 (픽셀)
    double rot1 = 0.0;   // robot-frame 이동 방향 (rad, 전진=0 좌측=+90도)
  };

  /// robot0 을 이동시키고 그 델타를 반환한다. (robot0_mutex 보유 상태에서 호출)
  Delta applyOdometry(SelfRobot &robot0, double ik_x, double ik_y,
                      bool foot_align) const;

  /// 델타 + IMU yaw 변화량으로 파티클을 전파한다. (pt_mutex 보유 상태에서 호출)
  void propagate(std::vector<Particle> &pt, int particle_num, const Delta &d,
                 double d_yaw) const;

  // 모션 노이즈 alpha 계수 (튜닝 가능)
  // alpha1: rot 노이즈 <- rot 크기
  // alpha2: rot 노이즈 <- trans 크기
  // alpha3: trans 노이즈 <- trans 크기
  // alpha4: trans 노이즈 <- rot 크기
  // alpha5: yaw 노이즈 <- yaw delta 크기
  //
  //   1단계: ALPHA3 부터 (위치 노이즈가 제일 체감된다)
  //          로봇 직진 -> 파티클 퍼짐이 실제 오차와 비슷하면 OK
  //   2단계: ALPHA1 (방향 노이즈)
  //          제자리 회전 -> 랜드마크 매칭 후 수렴 속도 확인
  //   3단계: ALPHA5 (yaw 노이즈)
  //          IMU 드리프트 구간에서 파티클 θ가 랜드마크로 보정되는지 확인
  //   4단계: ALPHA2, ALPHA4 는 마지막에 미세 조정
  static constexpr double ALPHA1 = 0.01;
  static constexpr double ALPHA2 = 0.001;
  static constexpr double ALPHA3 = 0.03;
  static constexpr double ALPHA4 = 0.01;
  static constexpr double ALPHA5 = 0.02;

  // 이 거리(픽셀) 미만이면 제자리로 보고 회전만 반영한다.
  static constexpr double STATIONARY_TRANS_PX = 0.1;
  // 제자리일 때 파티클에 주는 최소 위치 지터 (픽셀)
  static constexpr double STATIONARY_JITTER_PX = 0.1;
};
