#pragma once

#include "vision_interfaces/msg/robocupvisionfeature.hpp"

#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::inference
{

// The one place that must match the training conventions; gates and signs are
// export contracts, not tuning.
class ObservationEncoder
{
public:
  static Features fromMessage(
      const vision_interfaces::msg::Robocupvisionfeature& msg);

  // Training has no corner-arc T junctions, and near corners they would take the front T slots.
  // Uses body-frame geometry only, so it works while the robot is lost.
  static int dropCornerArcT(Features& features);

  Observation buildObs(const Features& features, double yaw_deg) const;

  static constexpr double CAM_H_FOV_DEG = 90.0;
  static constexpr double CAM_NEAR_M = 0.35;
  static constexpr double CAM_FAR_M = 6.0;
  static constexpr double CAM_FAR_GOALPOST_M = 4.0;

  // Refiner -> training convention (vx is left+). Changing these silently breaks the MLP.
  static constexpr double SIGN_RIGHT = -1.0;
  static constexpr double SIGN_FORWARD = 1.0;

  // yaw_deg has 0 = +y; the training heading has 0 = field +x, CCW.
  static constexpr double YAW_OFFSET_DEG = 90.0;

  // camera_yaw = yaw_deg + PAN_YAW_SIGN * deg(pan); +1 confirmed by measurement.
  static constexpr double PAN_YAW_SIGN = 1.0;
};

}
