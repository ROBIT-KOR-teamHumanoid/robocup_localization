#include "robocup_localization/app/pose_score.hpp"

#include <algorithm>
#include <limits>

#include "field_coordinate/field_coordinate.hpp"

namespace robocup_localization::app
{

namespace
{
constexpr double SEARCH_RADIUS_M = 0.5;
constexpr int SEARCH_HALF_STEPS = 2;
}

double bestPoseLogLikelihood(
    const core::PreparedObservations& prepared, double x_m, double y_m)
{
  const double step_m = SEARCH_RADIUS_M / static_cast<double>(SEARCH_HALF_STEPS);
  double best = -std::numeric_limits<double>::infinity();
  for (int ix = -SEARCH_HALF_STEPS; ix <= SEARCH_HALF_STEPS; ++ix)
  {
    for (int iy = -SEARCH_HALF_STEPS; iy <= SEARCH_HALF_STEPS; ++iy)
    {
      const double log_weight = core::MeasurementModel::logLikelihood(
          FieldCoord::toPixelX(x_m + ix * step_m), FieldCoord::toPixelY(y_m + iy * step_m),
          prepared);
      best = std::max(best, log_weight);
    }
  }
  return best;
}

}
