#pragma once

#include "robocup_localization/core/mcl/measurement_model.hpp"

namespace robocup_localization::app
{

// Best MeasurementModel log-likelihood within 0.5 m of (x_m, y_m), on a 5x5 grid,
// to absorb estimate error and PF jitter. Shared by kidnap and reloc verification.
double bestPoseLogLikelihood(
    const core::PreparedObservations& prepared, double x_m, double y_m);

}
