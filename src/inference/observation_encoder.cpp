#include "robocup_localization/inference/observation_encoder.hpp"

#include <algorithm>
#include <cmath>

#include "field_coordinate/field_coordinate.hpp"

namespace robocup_localization::inference
{

namespace
{
constexpr double ARC_RADIUS_TOL_M = 0.35;
constexpr double ARC_PERP_COS_MAX = 0.34;  // |cos| <= 0.34 means 70-110 deg

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

// fill_limit leaves never-trained slots empty (see FILL_* in inference_types.hpp).
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

}

Features ObservationEncoder::fromMessage(
    const vision_interfaces::msg::Robocupvisionfeature& msg)
{
  Features features;
  features.corner = pack(msg.corner_point_vec_x, msg.corner_point_vec_y);
  features.t_shape = pack(msg.t_point_vec_x, msg.t_point_vec_y);
  features.cross = pack(msg.cross_point_vec_x, msg.cross_point_vec_y);

  // pan comes from the Robocupvision message (QNode fills it); goalposts have bearings only.
  return features;
}

int ObservationEncoder::dropCornerArcT(Features& features)
{
  if (!FieldCoord::HAS_CORNER_ARC)
  {
    return 0;
  }
  if (features.corner.empty() || features.t_shape.size() < 2)
  {
    return 0;
  }

  const std::size_t t_count = features.t_shape.size();
  std::vector<char> is_arc(t_count, 0);

  for (const auto& corner : features.corner)
  {
    std::vector<std::size_t> candidate;
    std::vector<double> unit_x;
    std::vector<double> unit_y;

    for (std::size_t i = 0; i < t_count; ++i)
    {
      const double dx_m = (features.t_shape[i].vx_mm - corner.vx_mm) / 1000.0;
      const double dy_m = (features.t_shape[i].vy_mm - corner.vy_mm) / 1000.0;
      const double dist_m = std::hypot(dx_m, dy_m);
      if (dist_m < 1e-6)
      {
        continue;
      }
      if (std::fabs(dist_m - FieldCoord::L_ARC) > ARC_RADIUS_TOL_M)
      {
        continue;
      }
      candidate.push_back(i);
      unit_x.push_back(dx_m / dist_m);
      unit_y.push_back(dy_m / dist_m);
    }

    for (std::size_t a = 0; a + 1 < candidate.size(); ++a)
    {
      for (std::size_t b = a + 1; b < candidate.size(); ++b)
      {
        const double cos_ab = unit_x[a] * unit_x[b] + unit_y[a] * unit_y[b];
        if (std::fabs(cos_ab) <= ARC_PERP_COS_MAX)
        {
          is_arc[candidate[a]] = 1;
          is_arc[candidate[b]] = 1;
        }
      }
    }
  }

  std::vector<FeaturePoint> kept;
  kept.reserve(t_count);
  for (std::size_t i = 0; i < t_count; ++i)
  {
    if (!is_arc[i])
    {
      kept.push_back(features.t_shape[i]);
    }
  }

  const int dropped = static_cast<int>(t_count - kept.size());
  features.t_shape = std::move(kept);
  return dropped;
}

Observation ObservationEncoder::buildObs(
    const Features& features, double yaw_deg) const
{
  Observation obs{};

  // Training has no pan: undo the refiner's de-pan and use the camera heading so the FOV gate
  // and slots match training. The rotation sign is exact; PAN_YAW_SIGN was measured.
  const double pan = features.pan_rad;
  const double pan_cos = std::cos(pan);
  const double pan_sin = std::sin(pan);

  const double half_fov_rad = CAM_H_FOV_DEG * M_PI / 180.0 * 0.5;

  const auto convert =
      [half_fov_rad, pan_cos, pan_sin](
          const std::vector<FeaturePoint>& in, double far_m)
      {
        std::vector<std::array<double, 2>> out;
        out.reserve(in.size());
        for (const auto& point : in)
        {
          const double cam_vx = point.vx_mm * pan_cos - point.vy_mm * pan_sin;
          const double cam_vy = point.vx_mm * pan_sin + point.vy_mm * pan_cos;

          const double right = SIGN_RIGHT * (cam_vx / 1000.0);
          const double forward = SIGN_FORWARD * (cam_vy / 1000.0);

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
  offset += SLOT_GOALPOST * 3;

  const double cam_yaw_deg =
      yaw_deg + PAN_YAW_SIGN * (pan * 180.0 / M_PI);
  const double yaw_rad = (cam_yaw_deg + YAW_OFFSET_DEG) * M_PI / 180.0;
  obs[offset + 0] = static_cast<float>(std::sin(yaw_rad));
  obs[offset + 1] = static_cast<float>(std::cos(yaw_rad));

  return obs;
}

}
