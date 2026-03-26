/**
 * @file /include/robocup_localization25/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robocup_localization_QNODE_HPP_
#define robocup_localization_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <QStringListModel>
#include <string.h>
#include <mutex>

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "robocup_vision/msg/robocupvision.hpp"
#include "robocup_vision/msg/robocupvisionfeature.hpp"
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"

#include "robocup_localization/msg/robocup_localization.hpp"
#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/master2localization.hpp"

#include "../objects/robot.hpp"
#include "../objects/obstacle.hpp"
#include "../objects/ball.hpp"
#include "../objects/line_feature.hpp"

#define DEG2RAD (M_PI / 180)

struct VISIONMSG
{
  int Ball_cam_X;
  int Ball_cam_Y;
  float Ball_2d_X;
  float Ball_2d_Y;
  float Ball_D;
  float PAN;
  float TILT;
  float Ball_speed_X;
  float Ball_speed_Y;
  vector<float> ROBOT_VEC_X;
  vector<float> ROBOT_VEC_Y;
  int Ball_speed_level;
  int Scan_mode;
  int flag_pan;
};

struct GAMEMSG
{
  int mySide = 1;
  int penalty = 0;
  int position = 0;
  int robotNum = 0;
};

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char **argv);
  ~QNode();

  rclcpp::Publisher<robocup_localization::msg::RobocupLocalization>::SharedPtr localization_pub_;

  string robot_feature;

  int vision_data_cnt = 0;
  int vision_data_size = 0;

  // callback
  int vision_callback_timer = 0;

  // For print
  int robot_sight_flag = 0;
  int master_target_x = 0;
  int master_target_y = 0;

  // For UI
  int set_ball_flag = 0;

  // For Objects
  VISIONMSG visionMSG;
  ROBOT robot0 = ROBOT();
  ROBOT robot1 = ROBOT();
  ROBOT robot2 = ROBOT();
  ROBOT robot3 = ROBOT();
  ROBOT robot4 = ROBOT();
  BALL ball = BALL();
  BALL ball1 = BALL();
  BALL ball2 = BALL();
  BALL ball3 = BALL();
  BALL ball4 = BALL();
  Line_Feature Likelihood = Line_Feature();
  OBSTACLE obstacle0 = OBSTACLE();
  OBSTACLE obstacle1 = OBSTACLE();
  OBSTACLE obstacle2 = OBSTACLE();
  OBSTACLE obstacle3 = OBSTACLE();

  // For gamecontrol
  GAMEMSG gameMSG;

  // For particle
  int particle_num = 8000;               // config: particle_nums
  double weight_update_threshold = 14.0; // config: weight_update_threshold
  std::vector<ROBOT> pt;

  float particle_range = 0.0; // is 100

  // Thread safety: vision_point_vect 공유 보호
  std::mutex vision_mutex;
  // Thread safety: robot0 공유 보호 (ROS 콜백 스레드 ↔ Qt 메인 스레드)
  std::mutex robot0_mutex;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(const robocup_vision::msg::Robocupvision::SharedPtr msg);
  void visionFeatureCallback(const robocup_vision::msg::Robocupvisionfeature::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg);

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<robocup_vision::msg::Robocupvision>::SharedPtr vision_sub_;
  rclcpp::Subscription<robocup_vision::msg::Robocupvisionfeature>::SharedPtr vision_feature_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<gamecontroller::msg::Gamecontroldata>::SharedPtr game_control_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2localization>::SharedPtr master_sub_;

  // ikCallback 작업 변수
  double ik_Xmoved, ik_Ymoved;
  double ik_prev_x, ik_prev_y;
  double ik_dx, ik_dy;
  double ik_motion, ik_noise_sigma;
  static constexpr double IK_PX_MIN_X = FieldCoord::toPixelX(-FieldCoord::A / 2);
  static constexpr double IK_PX_MAX_X = FieldCoord::toPixelX(FieldCoord::A / 2);
  static constexpr double IK_PX_MIN_Y = FieldCoord::toPixelY(FieldCoord::B / 2);
  static constexpr double IK_PX_MAX_Y = FieldCoord::toPixelY(-FieldCoord::B / 2);
Q_SIGNALS:
  void rosShutDown();
signals:
  void featureReceived();
};

#endif
