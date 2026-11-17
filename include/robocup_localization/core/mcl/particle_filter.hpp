#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include "robocup_localization/core/mcl/mcl_pipeline.hpp"
#include "robocup_localization/core/types/particle.hpp"
#include "robocup_localization/core/types/vision_point.hpp"
#include "robocup_localization/core/mcl/observation_prefilter.hpp"

namespace robocup_localization::core
{

  struct ParticleFilterConfig
  {
    std::size_t particle_count = 8000;
    double min_fit = 0.3;  // best particle's geometric-mean likelihood per observation
    float particle_range = 0.0f;
    bool noise_enabled = true;
    int threads = 0;  // OpenMP threads for weighting, 0 = all cores
  };

  struct ParticleMotion
  {
    double translation_px = 0.0;
    double world_direction_rad = 0.0;
    double translation_stddev = 0.0;
    double direction_stddev = 0.0;
  };

  struct ParticleFilterSnapshot
  {
    std::vector<Particle> particles;
    std::vector<VISION_POINT> vision_points;
    PreFilter filter;
    std::vector<Particle> scored_particles;
    std::vector<double> scored_weights;
  };

  class ParticleFilter
  {
  public:
    explicit ParticleFilter(ParticleFilterConfig config = {});

    void configure(const ParticleFilterConfig &config);
    void setParticleRange(float particle_range);
    void setBallObservation(double relative_x_mm, double relative_y_mm);
    void setRobotYSign(int sign);
    int robotYSign() const;

    void submitFeatures(
        std::vector<VISION_POINT> observations,
        bool valid_observation,
        double robot_yaw_deg);

    MclResult correct(
        float robot_x, float robot_y, float robot_yaw_deg);

    void applyMotion(const ParticleMotion &motion);
    void randomize(float x, float y, float yaw_deg, float range);
    void mirrorY(float map_height);
    void clearVisionPoints();

    ParticleFilterSnapshot snapshot() const;
    std::size_t particleCount() const;

  private:
    mutable std::mutex mutex_;
    // Last snapshot, served while correct() holds mutex_ so the UI never stalls on it.
    mutable std::mutex snapshot_cache_mutex_;
    mutable ParticleFilterSnapshot snapshot_cache_;
    ParticleFilterConfig config_;
    MclPipeline pipeline_;
    std::vector<Particle> particles_;
  };

}
