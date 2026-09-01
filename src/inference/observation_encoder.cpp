/**
 * @file observation_encoder.cpp
 * @brief observation_encoder.hpp 구현 (ONNX 무관, 순수 좌표 변환)
 */

#include "robocup_localization/inference/observation_encoder.hpp"

#include <algorithm>
#include <cmath>

namespace robocup_localization::inference
{

namespace
{

/// 병렬 배열 두 개를 (x, y) 쌍 목록으로. 길이가 다르면 짧은 쪽에 맞춘다.
std::vector<FeaturePoint> pack(
    const std::vector<double>& vx, const std::vector<double>& vy)
{
  std::vector<FeaturePoint> out;
  const std::size_t n = std::min(vx.size(), vy.size());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    out.push_back({vx[i], vy[i]});
  }
  return out;
}

/// (right_m, forward_m) 벡터를 거리순으로 정렬해 [x,y,valid]*slot_count 슬롯에 채운다.
/// 슬롯이 남으면 valid=0 으로 패딩한다 (학습 시와 동일).
///
/// fill_limit: 실제로 채울 슬롯 수. 그 뒤 슬롯은 검출이 더 있어도 비워 둔다 —
/// 학습셋에서 한 번도 발화하지 않은 컬럼이라 표준화 스케일이 망가져 있다
/// (inference_types.hpp 의 FILL_* 주석). 정렬이 거리순이라 잘리는 것은 먼 검출이다.
void fillSlots(
    std::vector<std::array<double, 2>> points, int slot_count, int fill_limit,
    float* out)
{
  std::sort(
      points.begin(), points.end(),
      [](const std::array<double, 2>& a, const std::array<double, 2>& b)
      {
        return (a[0] * a[0] + a[1] * a[1]) < (b[0] * b[0] + b[1] * b[1]);
      });

  const int filled = std::min(
      {slot_count, fill_limit, static_cast<int>(points.size())});

  for (int i = 0; i < slot_count; ++i)
  {
    if (i < filled)
    {
      out[i * 3 + 0] = static_cast<float>(points[i][0]);
      out[i * 3 + 1] = static_cast<float>(points[i][1]);
      out[i * 3 + 2] = 1.0f;
    }
    else
    {
      out[i * 3 + 0] = 0.0f;
      out[i * 3 + 1] = 0.0f;
      out[i * 3 + 2] = 0.0f;
    }
  }
}

}  // namespace

ObservationEncoder::ObservationEncoder(double sign_right, double sign_forward)
    : sign_right_(sign_right), sign_forward_(sign_forward)
{
}

void ObservationEncoder::setSigns(double sign_right, double sign_forward)
{
  sign_right_ = sign_right;
  sign_forward_ = sign_forward;
}

void ObservationEncoder::setYawOffsetDeg(double yaw_offset_deg)
{
  yaw_offset_deg_ = yaw_offset_deg;
}

void ObservationEncoder::setPanYawSign(double pan_yaw_sign)
{
  pan_yaw_sign_ = pan_yaw_sign;
}

Features ObservationEncoder::fromMessage(
    const vision_interfaces::msg::Robocupvisionfeature& msg)
{
  Features features;
  features.corner = pack(msg.corner_point_vec_x, msg.corner_point_vec_y);
  features.t_shape = pack(msg.t_point_vec_x, msg.t_point_vec_y);
  features.cross = pack(msg.cross_point_vec_x, msg.cross_point_vec_y);
  features.pan_rad = msg.pan;

  if (msg.goal_post_x != 0.0 || msg.goal_post_y != 0.0)
  {
    features.goalpost.push_back({msg.goal_post_x, msg.goal_post_y});
  }
  return features;
}

Observation ObservationEncoder::buildObs(
    const Features& features, double yaw_deg) const
{
  Observation obs{};

  // ── 카메라 pan 보정 ────────────────────────────────────────────────
  // 학습에는 pan 이 없다. dataset_builder 의 world_to_body() 는 body yaw 하나로
  // 랜드마크를 옮기고, visible_mask() 의 FOV 게이트도 그 프레임의 정면 ±45deg
  // 를 쓴다. 즉 모델은 "내가 알려준 heading 기준 ±45deg 안의 랜드마크"만 본 적이
  // 있다.
  //
  // 배포에서는 refiner 가 point_vec_* 를 이미 body frame 으로 되돌려 놓는다:
  //     (absx, absy) = R(-P) * (cam_x, cam_y),  P = pan_rad
  // 그래서 머리를 돌리면 실제로 보이는 랜드마크가 body 기준 ±45deg 를 벗어나고,
  // 아래 FOV 게이트가 그것들을 버린다. 관측 집합이 한쪽으로 쏠린 채 모델에
  // 들어가는데, 학습에서는 거의 나오지 않는 배치다.
  //
  // 그래서 refiner 의 회전을 되돌려 카메라 프레임을 복원하고, heading 도 카메라
  // 것으로 바꿔 준다. 그러면 "카메라가 정면을 보는 로봇이 heading 만큼 돌아서
  // 있다" 가 되어 학습 상황과 정확히 일치한다. 위치는 어느 쪽을 보든 같으므로
  // 모델 출력의 의미는 변하지 않는다.
  //
  // 회전은 위 식의 역변환이라 부호가 확정이다:
  //     (cam_x, cam_y) = R(+P) * (absx, absy)
  // 반면 heading 쪽은 yaw_deg 가 CCW+ 인지에 달려 있다. yaw_offset_deg 90 을
  // 더하기만 해서 학습 규약(0deg=+x, CCW)으로 가는 걸 보면 yaw_deg 는 CCW+ 이고,
  // pan bearing 은 우측(CW)+ 이므로 camera_yaw = yaw - deg(P) 가 된다. 다만 이건
  // 코드에서 유도한 것이라 pan_yaw_sign_ 으로 뒤집을 수 있게 열어 둔다.
  const double pan = features.pan_rad;
  const double pan_cos = std::cos(pan);
  const double pan_sin = std::sin(pan);

  const double half_fov_rad = CAM_H_FOV_DEG * M_PI / 180.0 * 0.5;
  const double sign_right = sign_right_;
  const double sign_forward = sign_forward_;

  const auto convert =
      [sign_right, sign_forward, half_fov_rad, pan_cos, pan_sin](
          const std::vector<FeaturePoint>& in, double far_m)
      {
        std::vector<std::array<double, 2>> out;
        out.reserve(in.size());
        for (const auto& point : in)
        {
          // refiner 의 de-pan 을 되돌려 카메라 프레임으로.
          const double cam_vx = point.vx_mm * pan_cos - point.vy_mm * pan_sin;
          const double cam_vy = point.vx_mm * pan_sin + point.vy_mm * pan_cos;

          const double right = sign_right * (cam_vx / 1000.0);
          const double forward = sign_forward * (cam_vy / 1000.0);

          const double distance = std::hypot(right, forward);
          if (distance <= CAM_NEAR_M || distance >= far_m)
          {
            continue;
          }
          // 카메라 프레임이므로 이 게이트가 실제 화각과 일치한다.
          if (std::fabs(std::atan2(right, forward)) >= half_fov_rad)
          {
            continue;
          }

          out.push_back({right, forward});
        }
        return out;
      };

  int offset = 0;
  fillSlots(convert(features.corner, CAM_FAR_M), SLOT_CORNER, FILL_CORNER,
            obs.data() + offset);
  offset += SLOT_CORNER * 3;
  fillSlots(convert(features.t_shape, CAM_FAR_M), SLOT_T, FILL_T,
            obs.data() + offset);
  offset += SLOT_T * 3;
  fillSlots(convert(features.cross, CAM_FAR_M), SLOT_CROSS, FILL_CROSS,
            obs.data() + offset);
  offset += SLOT_CROSS * 3;
  fillSlots(convert(features.goalpost, CAM_FAR_GOALPOST_M), SLOT_GOALPOST,
            FILL_GOALPOST, obs.data() + offset);
  offset += SLOT_GOALPOST * 3;  // == OBS_VISION_SIZE (42)

  // 학습 규약(heading 0deg = 필드 +x, CCW)으로 옮긴 뒤 sin/cos 를 넣는다.
  // yaw_deg 자체는 0deg = +y 규약이라 그대로 쓰면 imu 만 90도 돌아간다.
  // 특징점을 카메라 프레임으로 옮겼으므로 heading 도 카메라 것을 넣는다.
  const double cam_yaw_deg =
      yaw_deg + pan_yaw_sign_ * (pan * 180.0 / M_PI);
  const double yaw_rad = (cam_yaw_deg + yaw_offset_deg_) * M_PI / 180.0;
  obs[offset + 0] = static_cast<float>(std::sin(yaw_rad));
  obs[offset + 1] = static_cast<float>(std::cos(yaw_rad));

  return obs;
}

}  // namespace robocup_localization::inference
