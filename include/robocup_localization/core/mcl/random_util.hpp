#pragma once

#include "robocup_localization/core/types/particle.hpp"

namespace robocup_localization::core
{

    inline constexpr double THETA_INIT_SPREAD_DEG = 0;

    double gaussianRandom(double average, double stdev);

    double uniformRandom(double upper);

    void randomizeParticle(
        Particle &particle, int x, int y, float particle_range, float theta);

}
