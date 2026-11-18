#include "robocup_localization/app/kidnap_recovery.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "robocup_localization/app/pose_score.hpp"
#include "robocup_localization/core/mcl/measurement_model.hpp"

namespace robocup_localization::app
{

namespace
{
double distance(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}
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

void KidnapRecovery::dropExpired(double now_sec)
{
  std::size_t first_live = 0;
  while (first_live < samples_.size() &&
         now_sec - samples_[first_live].t_sec > config_.window_sec)
  {
    ++first_live;
  }
  if (first_live > 0)
  {
    samples_.erase(samples_.begin(), samples_.begin() + first_live);
    frames_.erase(frames_.begin(), frames_.begin() + first_live);
  }
}

std::vector<std::size_t> KidnapRecovery::largestCluster() const
{
  std::vector<std::size_t> best;
  for (std::size_t seed = 0; seed < samples_.size(); ++seed)
  {
    std::vector<std::size_t> members;
    for (std::size_t i = 0; i < samples_.size(); ++i)
    {
      if (distance(samples_[i].x_m, samples_[i].y_m,
                   samples_[seed].x_m, samples_[seed].y_m) <= config_.cluster_radius_m)
      {
        members.push_back(i);
      }
    }
    if (members.size() > best.size())
    {
      best = std::move(members);
    }
  }
  return best;
}

void KidnapRecovery::keepOnly(const std::vector<std::size_t>& indices)
{
  std::vector<Sample> samples;
  std::vector<KidnapFrame> frames;
  samples.reserve(indices.size());
  frames.reserve(indices.size());
  for (const std::size_t i : indices)
  {
    samples.push_back(samples_[i]);
    frames.push_back(std::move(frames_[i]));
  }
  samples_ = std::move(samples);
  frames_ = std::move(frames);
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

  const core::PreparedObservations prepared = core::MeasurementModel::prepare(
      frame.observations, static_cast<float>(frame.yaw_deg), landmarks_, context);
  return bestPoseLogLikelihood(prepared, x_m, y_m);
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

  dropExpired(now_sec);

  // A blind frame says nothing either way, so it neither counts nor resets.
  if (!frame.mlp_valid)
  {
    return verdict;
  }

  // An agreeing estimate is evidence that the PF is fine.
  if (distance(frame.pf_x_m, frame.pf_y_m, frame.mlp_x_m, frame.mlp_y_m) <
      config_.disagree_dist_m)
  {
    samples_.clear();
    frames_.clear();
    return verdict;
  }

  samples_.push_back(Sample{now_sec, frame.mlp_x_m, frame.mlp_y_m});
  frames_.push_back(std::move(frame));

  if (static_cast<int>(samples_.size()) < config_.min_frames)
  {
    return verdict;
  }

  std::vector<std::size_t> cluster = largestCluster();
  if (static_cast<int>(cluster.size()) < config_.min_frames)
  {
    return verdict;
  }
  keepOnly(cluster);

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
