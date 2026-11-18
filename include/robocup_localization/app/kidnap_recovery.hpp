#pragma once

#include <cstddef>
#include <vector>

#include "robocup_localization/core/mcl/observation_prefilter.hpp"
#include "robocup_localization/core/types/vision_point.hpp"

namespace robocup_localization::app
{

struct KidnapRecoveryConfig
{
  bool enable = false;

  // Must be >= MclTune::MAX_JUMP_M; below that the PF still corrects itself.
  double disagree_dist_m = 2.0;

  double cluster_radius_m = 0.5;

  // Valid MLP frames needed inside the window. Not consecutive: only ~13% of frames see
  // >= feature_min landmarks and they flicker, so a consecutive streak rarely forms.
  int min_frames = 10;

  // Disagreeing samples older than this are dropped; blind frames no longer reset the count.
  double window_sec = 6.0;

  double cooldown_sec = 3.0;

  // The MLP can be consistently wrong (symmetric confusion), so clustering alone is not enough.
  bool verify = true;

  double verify_ratio = 1.5;

  // Geometric-mean likelihood per observation (0..1); the ratio alone means nothing near 0.
  double verify_min_score = 0.3;
};

struct KidnapFrame
{
  double pf_x_m = 0.0;
  double pf_y_m = 0.0;

  bool mlp_valid = false;
  double mlp_x_m = 0.0;
  double mlp_y_m = 0.0;

  double yaw_deg = 0.0;
  int robot_y_sign = 0;
  // Must be the frame the MLP saw: the MCL buffer mixes in synthetic observations
  // built around robot0, which biases scoring toward the current pose.
  std::vector<VISION_POINT> observations;
};

struct KidnapVerdict
{
  bool kidnapped = false;

  bool rejected = false;

  double x_m = 0.0;
  double y_m = 0.0;

  int frames = 0;
  double spread_m = 0.0;

  double candidate_score = 0.0;
  double current_score = 0.0;
};

class KidnapRecovery
{
public:
  void configure(const KidnapRecoveryConfig& config);
  const KidnapRecoveryConfig& config() const noexcept { return config_; }

  KidnapVerdict update(double now_sec, KidnapFrame frame);

  void reset();

  std::size_t pending() const noexcept { return samples_.size(); }

private:
  struct Sample
  {
    double t_sec = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
  };

  void dropExpired(double now_sec);

  // Indices of the largest group of samples within cluster_radius_m of one sample.
  // A stray estimate (symmetric confusion, noise) is left out instead of resetting.
  std::vector<std::size_t> largestCluster() const;

  void keepOnly(const std::vector<std::size_t>& indices);

  Sample centroid() const;

  double spread() const;

  double bestLogLikelihood(const KidnapFrame& frame, double x_m, double y_m) const;

  void verify(KidnapVerdict& verdict) const;

  void startCooldown(double now_sec);

  KidnapRecoveryConfig config_;
  std::vector<Sample> samples_;
  std::vector<KidnapFrame> frames_;
  double cooldown_until_sec_ = 0.0;
  bool cooldown_active_ = false;

  PreFilter landmarks_;
};

}
