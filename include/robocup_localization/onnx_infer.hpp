/**
 * @file onnx_infer.hpp
 * @brief 학습형 자기위치추정 ONNX 추론 엔진 (vision_imu MLP + clsreg 재위치추정)
 *
 * robocup_learned_localization 에서 export 한 두 ONNX 모델을 로드해
 * localization 프로세스 안에서 직접 추론한다. onnxruntime 헤더는 .cpp 로
 * 격리(PIMPL)하여 이 헤더는 순수 C++ 로 유지한다 (Qt MOC/qnode 안전).
 *
 * 입력 obs[44] 레이아웃 (모델 export json 과 동일):
 *   vision 42 = corner6 + t4 + cross2 + goalpost2, 각 [x=right(m), y=forward(m), valid]
 *   imu    2  = [sin(yaw), cos(yaw)]
 * 표준화는 ONNX 그래프에 내장되어 있으므로 "원시 관측"을 그대로 넣는다.
 */

#ifndef ROBOCUP_LOCALIZATION_ONNX_INFER_HPP_
#define ROBOCUP_LOCALIZATION_ONNX_INFER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace learned_loc
{

// vision_feature 메시지의 point_vec (mm) 한 점.
//   vx_mm = point_vec_x = 좌(+left) 방향 mm
//   vy_mm = point_vec_y = 전방(+forward) mm
struct FeaturePoint
{
  double vx_mm;
  double vy_mm;
};

struct LearnedConfig
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

// MLP 단일프레임 pose 추론 결과 (필드 프레임, 미터).
struct MlpResult
{
  bool valid = false;
  double x_m = 0.0;
  double y_m = 0.0;
};

// clsreg 재위치추정 후보 하나.
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

class OnnxLocalizer
{
public:
  OnnxLocalizer();
  ~OnnxLocalizer();

  OnnxLocalizer(const OnnxLocalizer &) = delete;
  OnnxLocalizer &operator=(const OnnxLocalizer &) = delete;

  // 설정에 따라 활성화된 모델만 로드한다. 실패 시 false + last_error().
  bool init(const LearnedConfig &config);

  bool mlp_ready() const;
  bool reloc_ready() const;

  const std::string &last_error() const;

  // vision_feature 원시 배열 + IMU yaw(deg) 로 obs[44] 조립.
  // goalpost 는 현재 소스가 없으면 빈 벡터를 넘겨 valid=0 으로 채운다.
  std::array<float, 44> buildObs(
      const std::vector<FeaturePoint> &corner,
      const std::vector<FeaturePoint> &t_shape,
      const std::vector<FeaturePoint> &cross,
      const std::vector<FeaturePoint> &goalpost,
      double yaw_deg) const;

  MlpResult runMlp(const std::array<float, 44> &obs);
  RelocResult runReloc(const std::array<float, 44> &obs);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace learned_loc

#endif  // ROBOCUP_LOCALIZATION_ONNX_INFER_HPP_
