#pragma once

#include <memory>
#include <string>

#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::inference
{

  class OnnxLocalizer
  {
  public:
    OnnxLocalizer();
    ~OnnxLocalizer();

    OnnxLocalizer(const OnnxLocalizer &) = delete;
    OnnxLocalizer &operator=(const OnnxLocalizer &) = delete;

    bool init(const InferenceConfig &config);

    bool mlpReady() const;
    bool relocReady() const;

    const std::string &lastError() const;

    MlpResult runMlp(const Observation &obs);
    RelocResult runReloc(const Observation &obs);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

}
