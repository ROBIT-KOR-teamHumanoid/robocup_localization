#pragma once

#include <string>

// 오도메트리 캘리브레이션 파일 (resources/param/PARAM{n}.txt)
// ============================================================
// GUI 슬라이더로 맞춘 값을 로봇별 파일에 저장/복원한다. 파일은 값 5개가
// 순서대로 한 줄씩 들어있는 단순 텍스트다 — 순서가 곧 형식이므로 여기 한
// 곳에서만 다룬다.
//
//   1) particle_range        재살포 sigma (픽셀)
//   2) odom_scale_forward    전진 배율
//   3) odom_scale_backward   후진
//   4) odom_scale_left       좌
//   5) odom_scale_right      우
//
// 로봇 번호는 robot_config.h 의 ROBIT_HUMANOID_ROBOT_NUMBER_{1..6} 로 정해진다.
// 예전에는 이 #ifdef 블록이 load 쪽과 save 쪽에 그대로 복붙돼 있었다.
namespace param_store
{

struct OdomParams
{
  float particle_range = 0.0f;
  float odom_scale_forward = 0.30f;
  float odom_scale_backward = 0.30f;
  float odom_scale_left = 0.30f;
  float odom_scale_right = 0.30f;
};

/// 이 로봇 번호에 해당하는 PARAM 파일 경로. 패키지 share 디렉터리 기준.
std::string filePath();

/// 파일에서 읽어 out 을 채운다. 파일이 없으면 false (out 은 그대로 둔다).
bool load(OdomParams &out);

/// 현재 값을 파일에 쓴다. 열기에 실패하면 false.
bool save(const OdomParams &params);

}  // namespace param_store
