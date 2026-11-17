#include "robocup_localization/app/kidnap_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "field_coordinate/field_coordinate.hpp"
#include "robocup_localization/core/mcl/measurement_model.hpp"

namespace robocup_localization::app
{

namespace
{
double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

// 5x5 grid search around each pose to absorb MLP error and PF jitter.
constexpr double SEARCH_RADIUS_M = 0.5;
constexpr int SEARCH_HALF_STEPS = 2;
}

void KidnapRecovery::configure(const KidnapRecoveryConfig& config)
{
  config_ = config;
  reset();
}

void KidnapRecovery::reset()
{
  samples_.clear();
  frames_.clear();
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

double KidnapRecovery::bestLogLikelihood(
    const KidnapFrame& frame, double x_m, double y_m) const
{
  // No ball association: the ball's field position is derived from robot0.
  core::MeasurementContext context;
  context.robot_y_sign = frame.robot_y_sign;

  const double step_m =
      SEARCH_RADIUS_M / static_cast<double>(SEARCH_HALF_STEPS);
  const core::PreparedObservations prepared = core::MeasurementModel::prepare(
      frame.observations, static_cast<float>(frame.yaw_deg), landmarks_, context);

  double best = -std::numeric_limits<double>::infinity();
  for (int ix = -SEARCH_HALF_STEPS; ix <= SEARCH_HALF_STEPS; ++ix)
  {
    for (int iy = -SEARCH_HALF_STEPS; iy <= SEARCH_HALF_STEPS; ++iy)
    {
      const double log_weight = core::MeasurementModel::logLikelihood(
          FieldCoord::toPixelX(x_m + ix * step_m), FieldCoord::toPixelY(y_m + iy * step_m),
          prepared);
      best = std::max(best, log_weight);
    }
  }
  return best;
}

void KidnapRecovery::verify(KidnapVerdict& verdict) const
{
  double candidate_sum = 0.0;
  double current_sum = 0.0;
  double confidence_sum = 0.0;

  // Score each frame at its own MLP / PF pose; the robot may have moved between frames.
  for (std::size_t i = 0; i < frames_.size(); ++i)
  {
    const KidnapFrame& frame = frames_[i];
    if (frame.observations.empty())
    {
      continue;
    }
    candidate_sum += bestLogLikelihood(frame, samples_[i].x_m, samples_[i].y_m);
    current_sum += bestLogLikelihood(frame, frame.pf_x_m, frame.pf_y_m);
    for (const auto& observation : frame.observations)
    {
      confidence_sum += observation.CONFIDENCE;
    }
  }

  if (confidence_sum <= 0.0)
  {
    verdict.rejected = true;
    return;
  }

  verdict.candidate_score = std::exp(candidate_sum / confidence_sum);
  verdict.current_score = std::exp(current_sum / confidence_sum);

  const bool strong_enough = verdict.candidate_score >= config_.verify_min_score;
  const bool better_than_current =
      verdict.candidate_score >= verdict.current_score * config_.verify_ratio;

  if (strong_enough && better_than_current)
  {
    verdict.kidnapped = true;
  }
  else
  {
    verdict.rejected = true;
  }
}

void KidnapRecovery::startCooldown(double now_sec)
{
  samples_.clear();
  frames_.clear();
  cooldown_until_sec_ = now_sec + config_.cooldown_sec;
  cooldown_active_ = true;
}

KidnapVerdict KidnapRecovery::update(double now_sec, KidnapFrame frame)
{
  KidnapVerdict verdict;

  if (!config_.enable)
  {
    return verdict;
  }

  // Right after a move the PF has not converged yet, so disagreement is no evidence.
  if (cooldown_active_)
  {
    if (now_sec < cooldown_until_sec_)
    {
      samples_.clear();
      frames_.clear();
      return verdict;
    }
    cooldown_active_ = false;
  }

  if (!frame.mlp_valid)
  {
    samples_.clear();
    frames_.clear();
    return verdict;
  }

  if (distance(frame.pf_x_m, frame.pf_y_m, frame.mlp_x_m, frame.mlp_y_m) <
      config_.disagree_dist_m)
  {
    samples_.clear();
    frames_.clear();
    return verdict;
  }

  samples_.push_back(Sample{frame.mlp_x_m, frame.mlp_y_m});
  frames_.push_back(std::move(frame));
  // Not clustered with earlier samples: one of them is wrong, so restart from this one.
  if (spread() > config_.cluster_radius_m)
  {
    const Sample latest_sample = samples_.back();
    KidnapFrame latest_frame = std::move(frames_.back());
    samples_.clear();
    frames_.clear();
    samples_.push_back(latest_sample);
    frames_.push_back(std::move(latest_frame));
    return verdict;
  }

  if (static_cast<int>(samples_.size()) < config_.min_frames)
  {
    return verdict;
  }

  const Sample center = centroid();
  verdict.x_m = center.x_m;
  verdict.y_m = center.y_m;
  verdict.frames = static_cast<int>(samples_.size());
  verdict.spread_m = spread();

  if (config_.verify)
  {
    verify(verdict);
  }
  else
  {
    verdict.kidnapped = true;
  }

  // Cool down even on rejection so the same wrong MLP output is not re-verified every frame.
  startCooldown(now_sec);

  return verdict;
}

}
