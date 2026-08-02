#include "utils/team_udp_receiver.hpp"

#include <cmath>

namespace team_udp_receiver
{
void receive(const humanoid_interfaces::msg::Udp2master::SharedPtr &msg,
             Teammate &robot, BALL &ball)
{
  robot.x     = msg->localx;
  robot.y     = msg->localy;
  robot.yaw_deg     = msg->localyaw;
  robot.freshness     = 1;
  robot.state = msg->robotcase;

  ball.x = msg->ballx;
  ball.y = msg->bally;
  ball.d = std::sqrt(std::pow(msg->localx - msg->ballx, 2) +
                     std::pow(msg->localy - msg->bally, 2));
}

void tick(Teammate &robot, BALL &ball, float decay)
{
  robot.freshness -= decay;
  if (robot.freshness <= 0)
  {
    robot.x     = 0;
    robot.y     = 0;
    robot.yaw_deg     = 0;
    robot.freshness     = 0;
    robot.state = 0;
    ball.x = 0;
    ball.y = 0;
    ball.d = 999999;
  }
}

bool selectNearestBall(BALL &own, double own_ball_dist_px,
                       const BALL &b1, const BALL &b2, const BALL &b3, const BALL &b4)
{
  const BALL *team[4] = {&b1, &b2, &b3, &b4};

  // 자신 비전 거리에서 시작해 팀 로봇들과 비교, 최소 거리 관측 선택.
  // (0,0) 공은 미감지 상태이므로 제외.
  double min_d = own_ball_dist_px;
  int    min_i = 0;
  for (int i = 0; i < 4; i++)
  {
    const BALL &tb = *team[i];
    if (min_d > tb.d && (tb.x != 0 || tb.y != 0))
    {
      min_d = tb.d;
      min_i = i + 1;
    }
  }

  // 최소 거리가 팀 로봇 것이고, 자신이 공을 충분히 오래 놓쳤을 때만 채택.
  if (min_i != 0 && own.noballcnt > 10)
  {
    own.x         = team[min_i - 1]->x;
    own.y         = team[min_i - 1]->y;
    own.noballcnt = 25;
    own.from_udp  = true;
    return true;
  }

  own.from_udp = false;
  return false;
}
}
