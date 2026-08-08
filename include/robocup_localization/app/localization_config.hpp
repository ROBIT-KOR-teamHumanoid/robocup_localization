#pragma once

namespace robocup_localization::app
{

  struct InferenceEnableConfig
  {
    bool mlp = false;
    bool reloc = false;
  };

  struct LoopRateConfig
  {
    double ros_spin_hz = 20.0;
    double ui_data_hz = 20.0;
    double ui_render_hz = 5.0;
  };

  struct LocalizationConfig
  {
    LoopRateConfig loop_rate;
    InferenceEnableConfig inference;
  };

} // namespace robocup_localization::app
