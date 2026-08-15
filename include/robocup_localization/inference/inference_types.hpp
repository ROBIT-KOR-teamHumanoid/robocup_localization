#pragma once

#include <array>
#include <string>
#include <vector>

namespace robocup_localization::inference
{

// ===== obs 슬롯 규약 =====
// 레이아웃(=obs 크기)은 모델이 정한 값이라 바꿀 수 없다.
inline constexpr int SLOT_CORNER = 6;
inline constexpr int SLOT_T = 4;
inline constexpr int SLOT_CROSS = 2;
inline constexpr int SLOT_GOALPOST = 2;

// ===== 실제로 채우는 슬롯 수 (학습 분포 상한) =====
// 레이아웃은 위 SLOT_* 그대로 두고, 그 뒤 슬롯은 valid=0 으로 남긴다.
//
// 학습셋(20260812_c9de87, 27만 프레임)의 슬롯별 발화율:
//   corner  0:79.6%  1:25.8%  2:0.31%  3~5:0.000%
//   t       0:77.6%  1:33.4%  2:0.07%  3  :0.000%
//   cross   0: 3.4%  1: 2.4%
//   goalpost0:22.2%  1     :0.000%   (두 골대는 22m 떨어져 동시관측 불가)
//
// 발화율 0 인 컬럼은 Standardizer 의 sd 가 0 이 되고, sd+1e-6 이 ONNX 그래프에
// 그대로 구워져 있다 → 실기에서 그 슬롯에 값이 들어오면 표준화 후 1e6 배로
// 증폭돼 첫 Linear 를 지배한다(나머지 43개 입력이 무의미해짐). 학습이 덜 된
// 정도가 아니라 출력이 그 한 슬롯으로 결정되는 고장이라, 아예 채우지 않는다.
//
// 슬롯은 거리순 정렬이므로 잘려나가는 것은 항상 가장 먼 검출이다.
inline constexpr int FILL_CORNER = 3;
inline constexpr int FILL_T = 3;
inline constexpr int FILL_CROSS = 2;
inline constexpr int FILL_GOALPOST = 1;

static_assert(
    FILL_CORNER <= SLOT_CORNER && FILL_T <= SLOT_T &&
        FILL_CROSS <= SLOT_CROSS && FILL_GOALPOST <= SLOT_GOALPOST,
    "채우는 슬롯 수는 레이아웃 슬롯 수를 넘을 수 없습니다.");

inline constexpr int OBS_VISION_SIZE =
    (SLOT_CORNER + SLOT_T + SLOT_CROSS + SLOT_GOALPOST) * 3;  // 42
inline constexpr int OBS_IMU_SIZE = 2;
inline constexpr int OBS_SIZE = OBS_VISION_SIZE + OBS_IMU_SIZE;  // 44

using Observation = std::array<float, OBS_SIZE>;

/// obs 의 valid 플래그(슬롯마다 [x, y, valid] 의 3번째)를 세어 실제 관측 개수를 낸다.
///
/// ※ vision_feature 메시지의 배열 길이로 세면 안 된다 — buildObs 의 카메라 게이트
///   (near/far/FOV)를 통과하지 못한 검출이 빠지므로, 메시지에 3개가 있어도 obs 의
///   valid 는 0 일 수 있다. 모델이 학습된 조건은 **obs 기준** 이다.
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

  /// obs 의 valid 슬롯이 이 값보다 적으면 추론을 건너뛴다.
  ///
  /// 모델은 "가시 특징점 >= feature_min" 프레임만으로 학습됐다(rejection sampling).
  /// 그 미만은 분포 밖이고, 학습형 회귀는 그때 필드 중앙 쪽으로 끌린 값을 낸다 —
  /// 무효로 두는 것이 낫다. 값은 export 산출물의 meta 와 맞춰야 한다:
  ///   models/vision_imu.onnx.json 의 "feature_min_gate"
  int feature_min = 2;

  // obs 좌표 부호 (학습 규약과 실기 vision 규약이 다르면 반전).
  //   right_m   = sign_right   * (vx_mm / 1000)   [기본 -1: right = -left]
  //   forward_m = sign_forward * (vy_mm / 1000)   [기본 +1]
  double sign_right = -1.0;
  double sign_forward = 1.0;

  /// (dataset.py 의 imu = [sin(yaw), cos(yaw)], forward = dx*cos+dy*sin).
  double yaw_offset_deg = 90.0;
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
