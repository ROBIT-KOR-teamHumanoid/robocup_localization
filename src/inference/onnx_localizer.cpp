#include "robocup_localization/inference/onnx_localizer.hpp"

// Compiled as a stub when ROBOCUP_LOCALIZATION_WITH_ONNX is undefined; init() then fails.
#ifdef ROBOCUP_LOCALIZATION_WITH_ONNX

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "field_coordinate/field_coordinate.hpp"

namespace robocup_localization::inference
{

namespace
{

struct SessionBundle
{
  std::unique_ptr<Ort::Session> session;
  std::string input_name;
  std::vector<std::string> output_names;
};

}

struct OnnxLocalizer::Impl
{
  InferenceConfig config;
  std::string error;

  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "robocup_inference"};
  Ort::MemoryInfo mem_info{
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  SessionBundle mlp;
  SessionBundle reloc;

  bool loadSession(const std::string& path, SessionBundle& bundle)
  {
    try
    {
      Ort::SessionOptions options;
      options.SetIntraOpNumThreads(1);
      options.SetGraphOptimizationLevel(
          GraphOptimizationLevel::ORT_ENABLE_ALL);

      bundle.session =
          std::make_unique<Ort::Session>(env, path.c_str(), options);

      Ort::AllocatorWithDefaultOptions allocator;

      auto input = bundle.session->GetInputNameAllocated(0, allocator);
      bundle.input_name = input.get();

      const std::size_t output_count = bundle.session->GetOutputCount();
      bundle.output_names.clear();
      for (std::size_t i = 0; i < output_count; ++i)
      {
        auto output = bundle.session->GetOutputNameAllocated(i, allocator);
        bundle.output_names.emplace_back(output.get());
      }
      return true;
    }
    catch (const std::exception& e)
    {
      error = std::string("failed to load '") + path + "': " + e.what();
      bundle.session.reset();
      return false;
    }
  }

  std::vector<Ort::Value> run(SessionBundle& bundle, const Observation& obs)
  {
    std::array<int64_t, 2> shape{1, OBS_SIZE};

    Ort::Value input = Ort::Value::CreateTensor<float>(
        mem_info,
        const_cast<float*>(obs.data()),
        obs.size(),
        shape.data(),
        shape.size());

    const char* input_names[] = {bundle.input_name.c_str()};

    std::vector<const char*> output_names;
    output_names.reserve(bundle.output_names.size());
    for (const auto& name : bundle.output_names)
    {
      output_names.push_back(name.c_str());
    }

    return bundle.session->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input,
        1,
        output_names.data(),
        output_names.size());
  }
};

OnnxLocalizer::OnnxLocalizer() : impl_(std::make_unique<Impl>()) {}
OnnxLocalizer::~OnnxLocalizer() = default;

bool OnnxLocalizer::init(const InferenceConfig& config)
{
  impl_->config = config;
  impl_->error.clear();

  bool ok = true;

  // 다른 필드에서 학습한 모델이면 쓰지 않는다 — 출력 좌표가 그 필드 기준이라
  // 엉뚱한 위치를 낸다. 모델은 필드마다 model/<field>/ 에 따로 둔다.
  const auto field_matches = [this](const std::string& path) {
    double a = 0.0;
    double b = 0.0;
    if (!readModelField(path, a, b))
    {
      impl_->error = path + ".json 에 학습 필드(A/B) 가 없다";
      return false;
    }
    if (std::abs(a - MODEL_FIELD_A) > 1e-6 || std::abs(b - MODEL_FIELD_B) > 1e-6)
    {
      impl_->error = path + " 는 " + std::to_string(a) + "x" + std::to_string(b)
          + " 필드용인데 이 빌드는 " + std::to_string(MODEL_FIELD_A) + "x"
          + std::to_string(MODEL_FIELD_B) + " 이다 (robot_config.h)";
      return false;
    }
    return true;
  };

  if (config.mlp_enable)
  {
    ok = field_matches(config.mlp_model_path)
        && impl_->loadSession(config.mlp_model_path, impl_->mlp) && ok;
  }

  if (config.reloc_enable)
  {
    ok = field_matches(config.reloc_model_path)
        && impl_->loadSession(config.reloc_model_path, impl_->reloc) && ok;
  }

  return ok;
}

bool OnnxLocalizer::mlpReady() const
{
  return impl_->mlp.session != nullptr;
}

bool OnnxLocalizer::relocReady() const
{
  return impl_->reloc.session != nullptr;
}

const std::string& OnnxLocalizer::lastError() const
{
  return impl_->error;
}

MlpResult OnnxLocalizer::runMlp(const Observation& obs)
{
  MlpResult result;
  if (!mlpReady())
  {
    return result;
  }

  try
  {
    auto outputs = impl_->run(impl_->mlp, obs);
    if (outputs.empty())
    {
      return result;
    }

    const float* pose = outputs[0].GetTensorData<float>();
    result.x_m = static_cast<double>(pose[0]) * (MODEL_FIELD_A / 2.0);
    result.y_m = static_cast<double>(pose[1]) * (MODEL_FIELD_B / 2.0);
    result.valid = true;
  }
  catch (const std::exception& e)
  {
    impl_->error = std::string("mlp run failed: ") + e.what();
  }
  return result;
}

RelocResult OnnxLocalizer::runReloc(const Observation& obs)
{
  RelocResult result;
  if (!relocReady())
  {
    return result;
  }

  try
  {
    auto outputs = impl_->run(impl_->reloc, obs);
    if (outputs.size() < 2)
    {
      return result;
    }

    const float* prob = outputs[0].GetTensorData<float>();
    const float* offsets = outputs[1].GetTensorData<float>();
    const double offset_x = static_cast<double>(offsets[0]);
    const double offset_y = static_cast<double>(offsets[1]);

    int topk = impl_->config.reloc_topk;
    topk = std::clamp(topk, 1, RELOC_GRID_CELLS);

    std::vector<int> indices(RELOC_GRID_CELLS);
    std::iota(indices.begin(), indices.end(), 0);
    std::partial_sort(
        indices.begin(), indices.begin() + topk, indices.end(),
        [prob](int a, int b) { return prob[a] > prob[b]; });

    result.candidates.reserve(topk);
    for (int i = 0; i < topk; ++i)
    {
      const int cell = indices[i];
      const int ix = cell % RELOC_GRID_NX;
      const int iy = cell / RELOC_GRID_NX;

      RelocCandidate candidate;
      candidate.x_m =
          ((ix + offset_x) / RELOC_GRID_NX) * MODEL_FIELD_A - MODEL_FIELD_A / 2.0;
      candidate.y_m =
          ((iy + offset_y) / RELOC_GRID_NY) * MODEL_FIELD_B - MODEL_FIELD_B / 2.0;
      candidate.prob = static_cast<double>(prob[cell]);
      result.candidates.push_back(candidate);
    }
    result.valid = true;
  }
  catch (const std::exception& e)
  {
    impl_->error = std::string("reloc run failed: ") + e.what();
  }
  return result;
}

}

#else

namespace robocup_localization::inference
{

struct OnnxLocalizer::Impl
{
  std::string error =
      "onnxruntime 없이 빌드됨 (WITH_INFERENCE=OFF)";
};

OnnxLocalizer::OnnxLocalizer() : impl_(std::make_unique<Impl>()) {}
OnnxLocalizer::~OnnxLocalizer() = default;

bool OnnxLocalizer::init(const InferenceConfig&) { return false; }
bool OnnxLocalizer::mlpReady() const { return false; }
bool OnnxLocalizer::relocReady() const { return false; }
const std::string& OnnxLocalizer::lastError() const { return impl_->error; }
MlpResult OnnxLocalizer::runMlp(const Observation&) { return {}; }
RelocResult OnnxLocalizer::runReloc(const Observation&) { return {}; }

}

#endif
