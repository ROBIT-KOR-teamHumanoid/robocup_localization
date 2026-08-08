#pragma once

#include <vector>

#include "robocup_localization/core/mcl/measurement_model.hpp"
#include "robocup_localization/core/types/particle.hpp"

namespace robocup_localization::core
{

    class Resampler
    {
    public:
        static void systematic(
            std::vector<Particle> &particles,
            const std::vector<MEASUREMENT> &sorted_weights,
            double total_weight,
            double noise_stddev);
    };

} // namespace robocup_localization::core
