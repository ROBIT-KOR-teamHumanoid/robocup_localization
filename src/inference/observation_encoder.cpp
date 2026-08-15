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

Features ObservationEncoder::fromMessage(
    const vision_interfaces::msg::Robocupvisionfeature& msg)
{
  Features features;
  features.corner = pack(msg.corner_point_vec_x, msg.corner_point_vec_y);
  features.t_shape = pack(msg.t_point_vec_x, msg.t_point_vec_y);
  features.cross = pack(msg.cross_point_vec_x, msg.cross_point_vec_y);

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

  const double half_fov_rad = CAM_H_FOV_DEG * M_PI / 180.0 * 0.5;
  const double sign_right = sign_right_;
  const double sign_forward = sign_forward_;

  const auto convert =
      [sign_right, sign_forward, half_fov_rad](
          const std::vector<FeaturePoint>& in, double far_m)
      {
        std::vector<std::array<double, 2>> out;
        out.reserve(in.size());
        for (const auto& point : in)
        {
          const double right = sign_right * (point.vx_mm / 1000.0);
          const double forward = sign_forward * (point.vy_mm / 1000.0);

          const double distance = std::hypot(right, forward);
          if (distance <= CAM_NEAR_M || distance >= far_m)
          {
            continue;
          }
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
  const double yaw_rad = (yaw_deg + yaw_offset_deg_) * M_PI / 180.0;
  obs[offset + 0] = static_cast<float>(std::sin(yaw_rad));
  obs[offset + 1] = static_cast<float>(std::cos(yaw_rad));

  return obs;
}

}  // namespace robocup_localization::inference
