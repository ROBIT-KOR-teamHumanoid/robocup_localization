#pragma once

namespace robocup_localization::app
{

struct LocalizationConfig
{
  double ros_spin_hz = 20.0;
  double ui_data_hz = 20.0;
  double ui_render_hz = 5.0;
};

}  // namespace robocup_localization::app
