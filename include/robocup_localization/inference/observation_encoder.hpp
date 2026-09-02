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

  /// imu 슬롯을 만들 때 yaw_deg 에 더할 각도(도). InferenceConfig 참조.
  void setYawOffsetDeg(double yaw_offset_deg);

  /// pan 을 yaw 에 더할 때의 부호. 특징점 회전은 refiner 공식의 정확한
  /// 역변환이라 부호가 확정이지만, yaw 쪽은 yaw_deg 가 CCW+ 인지 CW+ 인지에
  /// 달려 있어 실측으로 확인해야 한다. 기본값 -1.0 의 근거는 .cpp 참조.
  void setPanYawSign(double pan_yaw_sign);

  /// 메시지 한 프레임에서 타입별 특징점을 뽑는다.
  /// goalpost 는 비전이 좌표를 주지 않아 비워둔다 → obs 슬롯이 valid=0 으로 채워진다.
  static Features fromMessage(
      const vision_interfaces::msg::Robocupvisionfeature& msg);

  /// 코너 아크가 만드는 T 접합을 features.t_shape 에서 제거한다. 제거한 개수 반환.
  ///
  /// 학습셋 생성기(field_constants.py 의 build_features)에는 이 8개가 없다.
  /// 게다가 코너 근처에서는 아크 T 가 가장 가까운 T 라 fillSlots 의 거리순
  /// 정렬에서 앞 슬롯을 차지하고, 진짜 골라인 T 를 학습이 거의 안 된 뒤 슬롯
  /// (t slot 2 발화율 0.07%)으로 밀어내거나 FILL_T 밖으로 잘라낸다.
  ///
  /// 판정은 body frame 기하만 쓴다 — pose 추정이 필요 없어서 로봇이 길을 잃은
  /// 상태(= clsreg 재위치추정이 도는 바로 그 순간)에도 동작한다.
  /// L_ARC == 0 인 필드에서는 아무것도 하지 않는다.
  static int dropCornerArcT(Features& features);

  /// 학습의 visible_mask() 와 동일한 게이트를 적용해 obs[44] 를 조립한다.
  ///
  /// features.pan_rad 가 0 이 아니면 카메라 pan 을 보정한다 — 자세한 규약은
  /// .cpp 의 buildObs 주석 참조. 0 이면 보정 전과 완전히 같게 동작한다.
  Observation buildObs(const Features& features, double yaw_deg) const;

  static constexpr double CAM_H_FOV_DEG = 90.0;
  static constexpr double CAM_NEAR_M = 0.35;
  static constexpr double CAM_FAR_M = 6.0;
  static constexpr double CAM_FAR_GOALPOST_M = 4.0;

private:
  double sign_right_ = -1.0;
  double sign_forward_ = 1.0;
  double yaw_offset_deg_ = 90.0;
  double pan_yaw_sign_ = -1.0;
};

}  // namespace robocup_localization::inference
