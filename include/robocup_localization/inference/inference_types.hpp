#pragma once

#include <array>
#include <string>
#include <vector>

namespace robocup_localization::inference
{

// Slot layout (obs size) is fixed by the model.
inline constexpr int SLOT_CORNER = 6;
inline constexpr int SLOT_T = 4;
inline constexpr int SLOT_CROSS = 2;
inline constexpr int SLOT_GOALPOST = 2;

// Slots that never fired in training have sd ~ 0 baked into the ONNX graph (1e6 gain), so
// filling them lets one input dominate. Slots are distance-sorted, so the farthest are dropped.
inline constexpr int FILL_CORNER = 3;
inline constexpr int FILL_T = 3;
inline constexpr int FILL_CROSS = 2;
inline constexpr int FILL_GOALPOST = 1;

static_assert(
    FILL_CORNER <= SLOT_CORNER && FILL_T <= SLOT_T &&
        FILL_CROSS <= SLOT_CROSS && FILL_GOALPOST <= SLOT_GOALPOST,
    "채우는 슬롯 수는 레이아웃 슬롯 수를 넘을 수 없습니다.");

inline constexpr int OBS_VISION_SIZE =
    (SLOT_CORNER + SLOT_T + SLOT_CROSS + SLOT_GOALPOST) * 3;
inline constexpr int OBS_IMU_SIZE = 2;
inline constexpr int OBS_SIZE = OBS_VISION_SIZE + OBS_IMU_SIZE;

using Observation = std::array<float, OBS_SIZE>;

// Count obs valid flags, not message lengths: camera gates drop detections.
inline int countValidSlots(const Observation& obs)
{
  int n = 0;
  for (int i = 0; i < OBS_VISION_SIZE; i += 3)
  {
    if (obs[i + 2] > 0.5f)
    {
      ++n;
    }
  }
  return n;
}

// Must equal FieldCoord::A / B (static_assert in onnx_localizer.cpp).
inline constexpr double MODEL_FIELD_A = 22.0;
inline constexpr double MODEL_FIELD_B = 14.0;

inline constexpr int RELOC_GRID_NX = 44;
inline constexpr int RELOC_GRID_NY = 28;
inline constexpr int RELOC_GRID_CELLS = RELOC_GRID_NX * RELOC_GRID_NY;

// vx_mm = left+, vy_mm = forward+.
struct FeaturePoint
{
  double vx_mm = 0.0;
  double vy_mm = 0.0;
};

struct Features
{
  std::vector<FeaturePoint> corner;
  std::vector<FeaturePoint> t_shape;
  std::vector<FeaturePoint> cross;
  std::vector<FeaturePoint> goalpost;  // vision sends no coordinates; always empty

  // Pan the refiner used to de-pan point_vec_*; buildObs undoes it.
  double pan_rad = 0.0;
};

struct InferenceConfig
{
  bool mlp_enable = false;
  bool reloc_enable = false;

  std::string mlp_model_path;
  std::string reloc_model_path;

  int reloc_topk = 30;

  // Read from the MLP model meta (feature_min_gate), never a parameter.
  int feature_min = 2;
};

bool readFeatureMinGate(const std::string& model_path, int& feature_min);

struct MlpResult
{
  bool valid = false;
  double x_m = 0.0;
  double y_m = 0.0;
};

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

}
