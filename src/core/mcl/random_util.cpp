#include "robocup_localization/core/mcl/random_util.hpp"

#include <random>

namespace robocup_localization::core
{

namespace
{

std::mt19937& getRng()
{
    static thread_local std::mt19937 gen(std::random_device{}());
    return gen;
}

}

double gaussianRandom(double average, double stdev)
{
    static thread_local std::normal_distribution<double> dist(0.0, 1.0);
    return average + stdev * dist(getRng());
}

double uniformRandom(double upper)
{
    return std::uniform_real_distribution<double>(0.0, upper)(getRng());
}

void randomizeParticle(
    Particle& particle, int x, int y, float particle_range, float theta)
{
    particle.x = x + gaussianRandom(0, particle_range);
    particle.y = y + gaussianRandom(0, particle_range);
    particle.yaw_deg = theta + gaussianRandom(0, THETA_INIT_SPREAD_DEG);
}

}
