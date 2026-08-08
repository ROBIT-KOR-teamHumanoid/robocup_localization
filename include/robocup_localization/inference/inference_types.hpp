#pragma once

#include <array>
#include <string>
#include <vector>

namespace robocup_localization::inference
{

// ===== obs 슬롯 규약 =====
inline constexpr int SLOT_CORNER = 6;
inline constexpr int SLOT_T = 4;
inline constexpr int SLOT_CROSS = 2;
inline constexpr int SLOT_GOALPOST = 2;

inline constexpr int OBS_VISION_SIZE =
    (SLOT_CORNER + SLOT_T + SLOT_CROSS + SLOT_GOALPOST) * 3;  // 42
inline constexpr int OBS_IMU_SIZE = 2;
inline constexpr int OBS_SIZE = OBS_VISION_SIZE + OBS_IMU_SIZE;  // 44

using Observation = std::array<float, OBS_SIZE>;

// ===== 모델이 학습된 필드 치수 (m) =====
// vision_imu.onnx.json 의 to_world.A / to_world.B.
// FieldCoord::A/B 와 다르면 MLP 출력이 조용히 틀린 좌표를 낸다 —
// onnx_localizer.cpp 에서 static_assert 로 잡는다.
inline constexpr double MODEL_FIELD_A = 22.0;
inline constexpr double MODEL_FIELD_B = 14.0;

// ===== clsreg 재위치추정 격자 =====
inline constexpr int RELOC_GRID_NX = 44;
inline constexpr int RELOC_GRID_NY = 28;
inline constexpr int RELOC_GRID_CELLS = RELOC_GRID_NX * RELOC_GRID_NY;  // 1232

/// vision_feature 메시지의 point_vec (mm) 한 점.
///   vx_mm = point_vec_x = 좌(+left) 방향 mm
///   vy_mm = point_vec_y = 전방(+forward) mm
struct FeaturePoint
{
  double vx_mm = 0.0;
  double vy_mm = 0.0;
};

/// 모델 입력으로 넘길 타입별 특징점 묶음.
struct Features
{
  std::vector<FeaturePoint> corner;
  std::vector<FeaturePoint> t_shape;
  std::vector<FeaturePoint> cross;
  std::vector<FeaturePoint> goalpost;  // 비전이 좌표를 주지 않음 → 항상 빈 값
};

struct InferenceConfig
{
  bool mlp_enable = false;
  bool reloc_enable = false;

  std::string mlp_model_path;
  std::string reloc_model_path;

  int reloc_topk = 30;

  // obs 좌표 부호 (학습 규약과 실기 vision 규약이 다르면 반전).
  //   right_m   = sign_right   * (vx_mm / 1000)   [기본 -1: right = -left]
  //   forward_m = sign_forward * (vy_mm / 1000)   [기본 +1]
  double sign_right = -1.0;
  double sign_forward = 1.0;
};

/// MLP 단일프레임 pose 추론 결과 (필드 프레임, 미터).
struct MlpResult
{
  bool valid = false;
  double x_m = 0.0;
  double y_m = 0.0;
};

/// clsreg 재위치추정 후보 하나.
struct RelocCandidate
{
  double x_m = 0.0;
  double y_m = 0.0;
  double prob = 0.0;
};

struct RelocResult
{
  bool valid = false;
  std::vector<RelocCandidate> candidates;
};

}  // namespace robocup_localization::inference
