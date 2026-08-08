/**
 * @file observation_encoder.hpp
 * @brief vision_feature 메시지 → 모델 입력 obs[44] 조립
 *
 * ONNX에 의존하지 않는다 — onnxruntime 없이도 빌드/테스트가 된다.
 * MCL 경로의 visionFeatureCallback 과 대응된다: 같은 메시지를 각자 필요한
 * 형태로 푼다. MCL 은 world-frame 픽셀이 필요하고, 학습형은 로봇 기준 mm 원본을 쓴다.
 *
 * 이 클래스가 학습 규약(robocup_inferencealization/src/env/settings/
 * dataset_constants.py)과 짝을 맞추는 유일한 지점이다. 카메라 게이트 상수를
 * yaml 로 빼지 않는 이유도 같다 — 튜닝 값이 아니라 모델과 맞아야 하는 export 규약이다.
 */

#pragma once

#include "vision_interfaces/msg/robocupvisionfeature.hpp"

#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::inference
{

class ObservationEncoder
{
public:
  ObservationEncoder() = default;
  ObservationEncoder(double sign_right, double sign_forward);

  void setSigns(double sign_right, double sign_forward);

  /// 메시지 한 프레임에서 타입별 특징점을 뽑는다.
  /// goalpost 는 비전이 좌표를 주지 않아 비워둔다 → obs 슬롯이 valid=0 으로 채워진다.
  static Features fromMessage(
      const vision_interfaces::msg::Robocupvisionfeature& msg);

  /// 학습의 visible_mask() 와 동일한 게이트를 적용해 obs[44] 를 조립한다.
  Observation buildObs(const Features& features, double yaw_deg) const;

  // 카메라 가시성 게이트 — 학습 규약과 반드시 일치해야 한다.
  //   dataset_constants.py: CAM = dict(h_fov_deg=90.0, near=0.35, far=6.0,
  //                                    far_goalpost=12.0)
  //   preprocessing.py visible_mask(): |atan2(right, forward)| < h_fov/2,
  //                                    near < r < far  (goalpost 만 far_goalpost)
  // 학습 시 obs 슬롯에는 이 게이트를 통과한 랜드마크만 들어갔다. 실기 관측을
  // 그대로 넣으면 모델이 훈련 중 본 적 없는 원거리/시야밖 입력을 받게 된다.
  static constexpr double CAM_H_FOV_DEG = 90.0;
  static constexpr double CAM_NEAR_M = 0.35;
  static constexpr double CAM_FAR_M = 6.0;
  static constexpr double CAM_FAR_GOALPOST_M = 12.0;

private:
  double sign_right_ = -1.0;
  double sign_forward_ = 1.0;
};

}  // namespace robocup_localization::inference
