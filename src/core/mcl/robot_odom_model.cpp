/*****************************************************************************
** ROBOT class implementations
*****************************************************************************/

#include "robocup_localization/core/mcl/robot_odom_model.hpp"
#include "field_coordinate/field_coordinate.hpp"

ROBOT::ROBOT() {}

void ROBOT::appendTrail(double Xmoved, double Ymoved)
{
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : trail
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 이동 경로를 저장

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
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : x, y
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 해당 값으로 이동

  appendTrail(Xmoved, Ymoved);

  this->now_ik_param_x = Xmoved;
  this->now_ik_param_y = Ymoved;
  if (Xmoved > 0)
    Xmoved *= odom_fx;
  else
    Xmoved *= odom_bx;

  if (Ymoved > 0)
    Ymoved *= odom_ly;
  else
    Ymoved *= odom_ry;

  this->x += Xmoved * cos((this->yaw_deg + 90) * DEG2RAD) - Ymoved * sin((this->yaw_deg + 90) * DEG2RAD);
  this->y -= Xmoved * sin((this->yaw_deg + 90) * DEG2RAD) + Ymoved * cos((this->yaw_deg + 90) * DEG2RAD);

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

double ROBOT::yawStddev(double yaw_delta_deg)
{
  return ALPHA5 * std::fabs(yaw_delta_deg) + 0.1;
}
