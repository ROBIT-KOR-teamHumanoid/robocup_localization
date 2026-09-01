#include "robocup_localization/app/kidnap_recovery.hpp"

#include <cmath>

namespace robocup_localization::app
{

namespace
{
double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}
}  // namespace

void KidnapRecovery::configure(const KidnapRecoveryConfig& config)
{
  config_ = config;
  reset();
}

void KidnapRecovery::reset()
{
  samples_.clear();
  cooldown_until_sec_ = 0.0;
  cooldown_active_ = false;
}

KidnapRecovery::Sample KidnapRecovery::centroid() const
{
  Sample center;
  if (samples_.empty())
  {
    return center;
  }

  for (const auto& sample : samples_)
  {
    center.x_m += sample.x_m;
    center.y_m += sample.y_m;
  }
  center.x_m /= static_cast<double>(samples_.size());
  center.y_m /= static_cast<double>(samples_.size());
  return center;
}

double KidnapRecovery::spread() const
{
  if (samples_.size() < 2)
  {
    return 0.0;
  }

  const Sample center = centroid();
  double worst = 0.0;
  for (const auto& sample : samples_)
  {
    const double d = distance(sample.x_m, sample.y_m, center.x_m, center.y_m);
    if (d > worst)
    {
      worst = d;
    }
  }
  return worst;
}

KidnapVerdict KidnapRecovery::update(
    double now_sec,
    double pf_x_m, double pf_y_m,
    bool mlp_valid,
    double mlp_x_m, double mlp_y_m)
{
  KidnapVerdict verdict;

  if (!config_.enable)
  {
    return verdict;
  }

  // 쿨다운 중에는 누적하지 않는다. 자세를 막 옮겨 놓은 참이라 PF 가 아직
  // 새 자리에 수렴하지 못했고, 그 상태의 불일치는 납치의 근거가 아니다.
  if (cooldown_active_)
  {
    if (now_sec < cooldown_until_sec_)
    {
      samples_.clear();
      return verdict;
    }
    cooldown_active_ = false;
  }

  // MLP 가 이번 프레임을 못 냈다 (관측 부족 등). 연속이 끊겼으므로 버린다.
  if (!mlp_valid)
  {
    samples_.clear();
    return verdict;
  }

  // PF 와 충분히 가깝다 = 정상 추적 중. 누적을 버린다.
  if (distance(pf_x_m, pf_y_m, mlp_x_m, mlp_y_m) < config_.disagree_dist_m)
  {
    samples_.clear();
    return verdict;
  }

  // 여기까지 왔으면 "PF 와 멀리 떨어진 MLP 추정" 한 장이다.
  // 이전 누적과 뭉치는지 본다 — 뭉치지 않으면 둘 중 하나는 헛짚은 것이므로
  // 이번 샘플을 새 출발점으로 삼는다.
  samples_.push_back(Sample{mlp_x_m, mlp_y_m});
  if (spread() > config_.cluster_radius_m)
  {
    const Sample latest = samples_.back();
    samples_.clear();
    samples_.push_back(latest);
    return verdict;
  }

  if (static_cast<int>(samples_.size()) < config_.min_frames)
  {
    return verdict;
  }

  // 충분히 많이, 충분히 모여서, 충분히 멀리 — 납치로 본다.
  const Sample center = centroid();
  verdict.kidnapped = true;
  verdict.x_m = center.x_m;
  verdict.y_m = center.y_m;
  verdict.frames = static_cast<int>(samples_.size());
  verdict.spread_m = spread();

  samples_.clear();
  cooldown_until_sec_ = now_sec + config_.cooldown_sec;
  cooldown_active_ = true;

  return verdict;
}

}  // namespace robocup_localization::app
