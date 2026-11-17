#include "robocup_localization/core/mcl/robot_odom_model.hpp"
#include "field_coordinate/field_coordinate.hpp"

ROBOT::ROBOT() {}

void ROBOT::appendTrail(double Xmoved, double Ymoved)
{
  if (Xmoved != 0 || Ymoved != 0)
  {
    point.X = this->x;
    point.Y = this->y;
    trail.push_back(point);
    if (trail.size() > 10)
    {
      trail.erase(trail.begin());
    }
  }
  else
  {
    trail.clear();
  }
}

void ROBOT::move(double Xmoved, double Ymoved)
{
  appendTrail(Xmoved, Ymoved);

  this->now_ik_param_x = Xmoved;
  this->now_ik_param_y = Ymoved;

  double forward_m;
  double left_m;
  if (odom_input == OdometryInput::Metric)
  {
    forward_m = Xmoved / METRIC_UNITS_PER_METER;
    left_m = Ymoved / METRIC_UNITS_PER_METER;
  }
  else
  {
    // Odom coefficients are calibrated in map-x pixels, so meters use that scale.
    if (Xmoved > 0)
      forward_m = Xmoved * odom_fx / FieldCoord::PX_PER_M_X;
    else
      forward_m = Xmoved * odom_bx / FieldCoord::PX_PER_M_X;

    if (Ymoved > 0)
      left_m = Ymoved * odom_ly / FieldCoord::PX_PER_M_X;
    else
      left_m = Ymoved * odom_ry / FieldCoord::PX_PER_M_X;
  }

  // Rotate in meters, then scale per axis: x/y px/m differ (45.8 / 50 on LARGE) and one
  // shared factor skews motion by up to 9%. Map y grows downward.
  const double heading = (this->yaw_deg + 90) * DEG2RAD;
  const double world_dx_m = forward_m * cos(heading) - left_m * sin(heading);
  const double world_dy_m = forward_m * sin(heading) + left_m * cos(heading);

  this->x += world_dx_m * FieldCoord::PX_PER_M_X;
  this->y -= world_dy_m * FieldCoord::PX_PER_M_Y;

  if (this->x < FieldCoord::toPixelX(-FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(-FieldCoord::A / 2);
  if (this->x > FieldCoord::toPixelX(FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(FieldCoord::A / 2);
  if (this->y < FieldCoord::toPixelY(FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(FieldCoord::B / 2);
  if (this->y > FieldCoord::toPixelY(-FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(-FieldCoord::B / 2);
}

double ROBOT::translationStddev(double trans_px, double rot_rad)
{
  return ALPHA3 * trans_px + ALPHA4 * std::fabs(rot_rad);
}

double ROBOT::directionStddev(double trans_px, double rot_rad)
{
  return ALPHA1 * std::fabs(rot_rad) + ALPHA2 * trans_px;
}
