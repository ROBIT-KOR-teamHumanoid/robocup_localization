/*****************************************************************************
** ROBOT class implementations
*****************************************************************************/

#include "../../include/robot_odom_model/robot_odom_model.hpp"
#include "../../include/field_coordinate/field_coordinate.hpp"

ROBOT::ROBOT() {}

ROBOT::ROBOT(float x, float y)
{
  this->x = x;
  this->y = y;
}

void ROBOT::create_time_stamp(double Xmoved, double Ymoved)
{
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : TIME_STAMP
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 이동 경로를 저장

  if (Xmoved != 0 || Ymoved != 0)
  {
    time_t timer;
    timer = time(NULL);
    point.TIME = timer;
    point.X = this->x;
    point.Y = this->y;
    TIME_STAMP.push_back(point);
    if (TIME_STAMP.size() > 10)
    {
      TIME_STAMP.erase(TIME_STAMP.begin());
    }
  }
  else
  {
    TIME_STAMP.clear();
  }
}

void ROBOT::move(double Xmoved, double Ymoved)
{
  // PRE CONDITION : Xmoved, Ymoved
  // POST CONDITION : x, y
  // PURPOSE : 로봇에 Xmoved 혹은 Ymoved값이 들어오면 해당 값으로 이동

  create_time_stamp(Xmoved, Ymoved);

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

  this->x += Xmoved * cos((this->z + 90) * DEG2RAD) - Ymoved * sin((this->z + 90) * DEG2RAD);
  this->y -= Xmoved * sin((this->z + 90) * DEG2RAD) + Ymoved * cos((this->z + 90) * DEG2RAD);

  if (this->x < FieldCoord::toPixelX(-FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(-FieldCoord::A / 2);
  if (this->x > FieldCoord::toPixelX(FieldCoord::A / 2))
    this->x = FieldCoord::toPixelX(FieldCoord::A / 2);
  if (this->y < FieldCoord::toPixelY(FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(FieldCoord::B / 2);
  if (this->y > FieldCoord::toPixelY(-FieldCoord::B / 2))
    this->y = FieldCoord::toPixelY(-FieldCoord::B / 2);
}
