/**
 * @file onnx_infer.cpp
 * @brief mlp_onnx_infer.hpp 구현 (onnxruntime C++ 격리)
 */

#include "mlp_localization/mlp_onnx_infer.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <onnxruntime_cxx_api.h>

namespace learned_loc
{

namespace
{
// 모델 export 규약 (robocup_learned_localization / LARGE 필드)
constexpr double FIELD_A = 22.0;   // 길이(m)
constexpr double FIELD_B = 14.0;   // 너비(m)
constexpr double FIELD_A2 = FIELD_A / 2.0;
constexpr double FIELD_B2 = FIELD_B / 2.0;

// clsreg 격자
constexpr int GRID_NX = 44;
constexpr int GRID_NY = 28;
constexpr int GRID_CELLS = GRID_NX * GRID_NY;  // 1232

constexpr int SLOT_CORNER = 6;
constexpr int SLOT_T = 4;
constexpr int SLOT_CROSS = 2;
constexpr int SLOT_GOALPOST = 2;

// 카메라 가시성 게이트 — 학습 규약과 반드시 일치해야 한다.
//   robocup_learned_localization/src/env/settings/dataset_constants.py
//     CAM = dict(h_fov_deg=90.0, near=0.35, far=6.0, far_goalpost=12.0)
//   preprocessing.py visible_mask(): |atan2(right, forward)| < h_fov/2,
//                                    near < r < far  (goalpost 만 far_goalpost)
// 학습 시 obs 슬롯에는 이 게이트를 통과한 랜드마크만 들어갔다. 실기 관측을
// 그대로 넣으면 모델이 훈련 중 본 적 없는 원거리/시야밖 입력을 받게 된다.
// yaml 로 빼지 않는 이유: 튜닝 값이 아니라 모델과 짝이 맞아야 하는 export 규약.
constexpr double CAM_H_FOV_DEG = 90.0;
constexpr double CAM_NEAR_M = 0.35;
constexpr double CAM_FAR_M = 6.0;
constexpr double CAM_FAR_GOALPOST_M = 12.0;

// 하나의 세션 + 입출력 이름
struct SessionBundle
{
  std::unique_ptr<Ort::Session> session;
  std::string input_name;
  std::vector<std::string> output_names;
};
}  // namespace

struct OnnxLocalizer::Impl
{
  LearnedConfig config;
  std::string error;

  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "learned_loc"};
  Ort::MemoryInfo mem_info{
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  SessionBundle mlp;
  SessionBundle reloc;

  bool loadSession(const std::string &path, SessionBundle &bundle)
  {
    try
    {
      Ort::SessionOptions opts;
      opts.SetIntraOpNumThreads(1);
      opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

      bundle.session =
          std::make_unique<Ort::Session>(env, path.c_str(), opts);

      Ort::AllocatorWithDefaultOptions alloc;

      auto in = bundle.session->GetInputNameAllocated(0, alloc);
      bundle.input_name = in.get();

      const size_t out_count = bundle.session->GetOutputCount();
      bundle.output_names.clear();
      for (size_t i = 0; i < out_count; ++i)
      {
        auto out = bundle.session->GetOutputNameAllocated(i, alloc);
        bundle.output_names.emplace_back(out.get());
      }
      return true;
    }
    catch (const std::exception &e)
    {
      error = std::string("failed to load '") + path + "': " + e.what();
      bundle.session.reset();
      return false;
    }
  }

  // obs[44] → 세션 실행, 출력 텐서들을 반환
  std::vector<Ort::Value> run(SessionBundle &bundle,
                              const std::array<float, 44> &obs)
  {
    std::array<int64_t, 2> shape{1, 44};

    Ort::Value input = Ort::Value::CreateTensor<float>(
        mem_info,
        const_cast<float *>(obs.data()),
        obs.size(),
        shape.data(),
        shape.size());

    const char *in_names[] = {bundle.input_name.c_str()};

    std::vector<const char *> out_names;
    out_names.reserve(bundle.output_names.size());
    for (const auto &n : bundle.output_names)
      out_names.push_back(n.c_str());

    return bundle.session->Run(
        Ort::RunOptions{nullptr},
        in_names,
        &input,
        1,
        out_names.data(),
        out_names.size());
  }
};

OnnxLocalizer::OnnxLocalizer() : impl_(std::make_unique<Impl>()) {}
OnnxLocalizer::~OnnxLocalizer() = default;

bool OnnxLocalizer::init(const LearnedConfig &config)
{
  impl_->config = config;
  impl_->error.clear();

  bool ok = true;

  if (config.mlp_enable)
    ok = impl_->loadSession(config.mlp_model_path, impl_->mlp) && ok;

  if (config.reloc_enable)
    ok = impl_->loadSession(config.reloc_model_path, impl_->reloc) && ok;

  return ok;
}

bool OnnxLocalizer::mlp_ready() const
{
  return impl_->mlp.session != nullptr;
}

bool OnnxLocalizer::reloc_ready() const
{
  return impl_->reloc.session != nullptr;
}

const std::string &OnnxLocalizer::last_error() const
{
  return impl_->error;
}

namespace
{
// (right_m, forward_m) 벡터를 거리순으로 정렬해 [x,y,valid]*n_slots 슬롯에 채운다.
void fillSlots(std::vector<std::array<double, 2>> pts,
               int n_slots,
               float *out)
{
  std::sort(pts.begin(), pts.end(),
            [](const std::array<double, 2> &a, const std::array<double, 2> &b) {
              return (a[0] * a[0] + a[1] * a[1]) < (b[0] * b[0] + b[1] * b[1]);
            });

  const int n = std::min<int>(n_slots, static_cast<int>(pts.size()));
  for (int i = 0; i < n_slots; ++i)
  {
    if (i < n)
    {
      out[i * 3 + 0] = static_cast<float>(pts[i][0]);
      out[i * 3 + 1] = static_cast<float>(pts[i][1]);
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

std::array<float, 44> OnnxLocalizer::buildObs(
    const std::vector<FeaturePoint> &corner,
    const std::vector<FeaturePoint> &t_shape,
    const std::vector<FeaturePoint> &cross,
    const std::vector<FeaturePoint> &goalpost,
    double yaw_deg) const
{
  std::array<float, 44> obs{};

  const double sr = impl_->config.sign_right;
  const double sf = impl_->config.sign_forward;

  // 학습의 visible_mask() 와 동일한 게이트를 적용하고 (right, forward) 로 변환.
  const double half_fov_rad = CAM_H_FOV_DEG * M_PI / 180.0 * 0.5;

  auto convert = [sr, sf, half_fov_rad](const std::vector<FeaturePoint> &in,
                                        double far_m) {
    std::vector<std::array<double, 2>> out;
    out.reserve(in.size());
    for (const auto &p : in)
    {
      const double right = sr * (p.vx_mm / 1000.0);
      const double forward = sf * (p.vy_mm / 1000.0);

      const double r = std::hypot(right, forward);
      if (r <= CAM_NEAR_M || r >= far_m)
        continue;
      if (std::fabs(std::atan2(right, forward)) >= half_fov_rad)
        continue;

      out.push_back({right, forward});
    }
    return out;
  };

  int off = 0;
  fillSlots(convert(corner, CAM_FAR_M), SLOT_CORNER, obs.data() + off);
  off += SLOT_CORNER * 3;
  fillSlots(convert(t_shape, CAM_FAR_M), SLOT_T, obs.data() + off);
  off += SLOT_T * 3;
  fillSlots(convert(cross, CAM_FAR_M), SLOT_CROSS, obs.data() + off);
  off += SLOT_CROSS * 3;
  fillSlots(convert(goalpost, CAM_FAR_GOALPOST_M), SLOT_GOALPOST,
            obs.data() + off);
  off += SLOT_GOALPOST * 3;  // == 42

  const double yaw = yaw_deg * M_PI / 180.0;
  obs[off + 0] = static_cast<float>(std::sin(yaw));
  obs[off + 1] = static_cast<float>(std::cos(yaw));

  return obs;
}

MlpResult OnnxLocalizer::runMlp(const std::array<float, 44> &obs)
{
  MlpResult result;
  if (!mlp_ready())
    return result;

  try
  {
    auto outputs = impl_->run(impl_->mlp, obs);
    if (outputs.empty())
      return result;

    const float *pose = outputs[0].GetTensorData<float>();
    result.x_m = static_cast<double>(pose[0]) * FIELD_A2;
    result.y_m = static_cast<double>(pose[1]) * FIELD_B2;
    result.valid = true;
  }
  catch (const std::exception &e)
  {
    impl_->error = std::string("mlp run failed: ") + e.what();
  }
  return result;
}

RelocResult OnnxLocalizer::runReloc(const std::array<float, 44> &obs)
{
  RelocResult result;
  if (!reloc_ready())
    return result;

  try
  {
    auto outputs = impl_->run(impl_->reloc, obs);
    if (outputs.size() < 2)
      return result;

    const float *prob = outputs[0].GetTensorData<float>();
    const float *offv = outputs[1].GetTensorData<float>();
    const double off_x = static_cast<double>(offv[0]);
    const double off_y = static_cast<double>(offv[1]);

    int topk = impl_->config.reloc_topk;
    if (topk < 1)
      topk = 1;
    if (topk > GRID_CELLS)
      topk = GRID_CELLS;

    std::vector<int> idx(GRID_CELLS);
    std::iota(idx.begin(), idx.end(), 0);
    std::partial_sort(
        idx.begin(), idx.begin() + topk, idx.end(),
        [prob](int a, int b) { return prob[a] > prob[b]; });

    result.candidates.reserve(topk);
    for (int i = 0; i < topk; ++i)
    {
      const int c = idx[i];
      const int ix = c % GRID_NX;
      const int iy = c / GRID_NX;

      RelocCandidate cand;
      cand.x_m = ((ix + off_x) / GRID_NX) * FIELD_A - FIELD_A2;
      cand.y_m = ((iy + off_y) / GRID_NY) * FIELD_B - FIELD_B2;
      cand.prob = static_cast<double>(prob[c]);
      result.candidates.push_back(cand);
    }
    result.valid = true;
  }
  catch (const std::exception &e)
  {
    impl_->error = std::string("reloc run failed: ") + e.what();
  }
  return result;
}

}  // namespace learned_loc
