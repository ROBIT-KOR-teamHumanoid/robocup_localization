#include "robocup_localization/core/mcl/resampler.hpp"

#include "robocup_localization/core/mcl/random_util.hpp"

namespace robocup_localization::core
{

void Resampler::systematic(
    std::vector<Particle>& particles,
    const std::vector<MEASUREMENT>& sorted_weights,
    double total_weight,
    double noise_stddev)
{
    const int particle_count = static_cast<int>(particles.size());
    if (particle_count == 0 ||
        static_cast<int>(sorted_weights.size()) < particle_count ||
        total_weight <= 0.0)
    {
        return;
    }

    std::vector<double> cdf(particle_count);
    cdf[0] = sorted_weights[0].WEIGHT / total_weight;
    for (int i = 1; i < particle_count; ++i)
    {
        cdf[i] = cdf[i - 1] + sorted_weights[i].WEIGHT / total_weight;
    }

    const double step  = 1.0 / particle_count;
    const double start = uniformRandom(step);

    std::vector<Particle> selected(particle_count);
    int j = 0;
    for (int i = 0; i < particle_count; ++i)
    {
        const double target = start + i * step;
        while (j < particle_count - 1 && cdf[j] < target)
        {
            ++j;
        }
        selected[i] = particles[sorted_weights[j].NUM];
    }

    for (int i = 0; i < particle_count; ++i)
    {
        particles[i].x = selected[i].x + gaussianRandom(0, noise_stddev);
        particles[i].y = selected[i].y + gaussianRandom(0, noise_stddev);
        particles[i].yaw_deg = selected[i].yaw_deg;
    }
}

}
