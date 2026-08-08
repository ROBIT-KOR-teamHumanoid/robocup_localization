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

    // 설정에서 활성화된 모델만 로드한다. 실패 시 false + last_error().
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

} // namespace robocup_localization::inference
