#pragma once

#include <deque>
#include <vector>

#include "robocup_localization/core/mcl/measurement_model.hpp"
#include "robocup_localization/core/mcl/observation_prefilter.hpp"
#include "robocup_localization/core/types/vision_point.hpp"

namespace robocup_localization::app
{

// Global relocalization from the ClsReg proposer (clsreg.onnx).
// The MCL cannot jump beyond MAX_JUMP_M, so once it settles on the wrong spot it stays there.
// When the current pose stops explaining the recent observations, every ClsReg candidate of
// the last window_sec is carried to the present with odometry and scored as a pose track
// against the same observations; a clearly better track replaces the pose.
struct RelocRecoveryConfig
{
  bool enable = false;

  double window_sec = 3.0;

  // Geometric-mean likelihood per observation of the current pose track (0..1).
  // Simulation: a correct pose scores p10 0.91, a lost one p90 0.25.
  double lost_fit = 0.3;

  // A track scored on fewer observations is too ambiguous to judge.
  int min_observations = 4;

  double verify_min_score = 0.3;
  double verify_ratio = 1.5;

  // Candidates this close are the MCL's own business.
  double min_jump_m = 1.0;

  double cooldown_sec = 5.0;
};

struct RelocProposal
{
  double x_m = 0.0;
  double y_m = 0.0;
};

struct RelocFrame
{
  double pf_x_m = 0.0;
  double pf_y_m = 0.0;

  // Cumulative odometry in field meters; only differences between frames are used.
  double odom_x_m = 0.0;
  double odom_y_m = 0.0;

  double yaw_deg = 0.0;
  int robot_y_sign = 0;  // goalpost half-field vote, see PreFilter::computeYSign
  std::vector<VISION_POINT> observations;
  std::vector<RelocProposal> proposals;
};

struct RelocVerdict
{
  bool relocalize = false;
  bool lost = false;

  double x_m = 0.0;
  double y_m = 0.0;

  double candidate_score = 0.0;
  double current_score = 0.0;
  int proposals = 0;
};

class RelocRecovery
{
public:
  void configure(const RelocRecoveryConfig& config);
  const RelocRecoveryConfig& config() const noexcept { return config_; }

  RelocVerdict update(double now_sec, RelocFrame frame);

  // After any teleport: the buffered frames describe the old pose track.
  void holdOff(double now_sec);

  void reset();

private:
  struct Entry
  {
    double t_sec = 0.0;
    double odom_x_m = 0.0;
    double odom_y_m = 0.0;
    core::PreparedObservations prepared;
    double confidence = 0.0;
    int observation_count = 0;
    std::vector<RelocProposal> proposals;
  };

  // Score of the track that ends at (x_m, y_m) now; -1 when nothing was observed.
  double trackScore(double x_m, double y_m, double odom_x_m, double odom_y_m) const;

  RelocRecoveryConfig config_;
  std::deque<Entry> entries_;
  double cooldown_until_sec_ = -1.0;

  PreFilter landmarks_;
};

}
