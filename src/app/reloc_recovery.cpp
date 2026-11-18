#include "robocup_localization/app/reloc_recovery.hpp"

#include <cmath>
#include <utility>

#include "robocup_localization/app/pose_score.hpp"

namespace robocup_localization::app
{

void RelocRecovery::configure(const RelocRecoveryConfig& config)
{
  config_ = config;
  reset();
}

void RelocRecovery::reset()
{
  entries_.clear();
  cooldown_until_sec_ = -1.0;
}

void RelocRecovery::holdOff(double now_sec)
{
  entries_.clear();
  cooldown_until_sec_ = now_sec + config_.cooldown_sec;
}

double RelocRecovery::trackScore(
    double x_m, double y_m, double odom_x_m, double odom_y_m) const
{
  double log_sum = 0.0;
  double confidence_sum = 0.0;
  for (const Entry& entry : entries_)
  {
    if (entry.observation_count == 0)
    {
      continue;
    }
    log_sum += bestPoseLogLikelihood(
        entry.prepared,
        x_m - (odom_x_m - entry.odom_x_m),
        y_m - (odom_y_m - entry.odom_y_m));
    confidence_sum += entry.confidence;
  }
  return confidence_sum > 0.0 ? std::exp(log_sum / confidence_sum) : -1.0;
}

RelocVerdict RelocRecovery::update(double now_sec, RelocFrame frame)
{
  RelocVerdict verdict;
  if (!config_.enable)
  {
    return verdict;
  }

  core::MeasurementContext context;
  context.robot_y_sign = frame.robot_y_sign;

  Entry entry;
  entry.t_sec = now_sec;
  entry.odom_x_m = frame.odom_x_m;
  entry.odom_y_m = frame.odom_y_m;
  entry.prepared = core::MeasurementModel::prepare(
      frame.observations, static_cast<float>(frame.yaw_deg), landmarks_, context);
  entry.observation_count = static_cast<int>(frame.observations.size());
  for (const auto& observation : frame.observations)
  {
    entry.confidence += observation.CONFIDENCE;
  }
  entry.proposals = std::move(frame.proposals);
  entries_.push_back(std::move(entry));

  while (!entries_.empty() && now_sec - entries_.front().t_sec > config_.window_sec)
  {
    entries_.pop_front();
  }

  if (frame.observations.empty() || now_sec < cooldown_until_sec_)
  {
    return verdict;
  }

  int observation_count = 0;
  for (const Entry& e : entries_)
  {
    observation_count += e.observation_count;
  }
  if (observation_count < config_.min_observations)
  {
    return verdict;
  }

  verdict.current_score =
      trackScore(frame.pf_x_m, frame.pf_y_m, frame.odom_x_m, frame.odom_y_m);
  if (verdict.current_score >= config_.lost_fit)
  {
    return verdict;
  }
  verdict.lost = true;

  double best = -1.0;
  for (const Entry& e : entries_)
  {
    for (const RelocProposal& proposal : e.proposals)
    {
      const double x_m = proposal.x_m + (frame.odom_x_m - e.odom_x_m);
      const double y_m = proposal.y_m + (frame.odom_y_m - e.odom_y_m);
      const double score = trackScore(x_m, y_m, frame.odom_x_m, frame.odom_y_m);
      ++verdict.proposals;
      if (score > best)
      {
        best = score;
        verdict.x_m = x_m;
        verdict.y_m = y_m;
      }
    }
  }
  verdict.candidate_score = best;

  const bool strong_enough = best >= config_.verify_min_score;
  const bool better_than_current = best >= verdict.current_score * config_.verify_ratio;
  const bool far_enough =
      std::hypot(verdict.x_m - frame.pf_x_m, verdict.y_m - frame.pf_y_m) > config_.min_jump_m;
  if (strong_enough && better_than_current && far_enough)
  {
    verdict.relocalize = true;
    holdOff(now_sec);
  }
  return verdict;
}

}
