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
#include <thread>
#include <condition_variable>

#include "humanoid_interfaces/msg/imu_msg.hpp"
#include "vision_interfaces/msg/robocupvision.hpp"
#include "vision_interfaces/msg/robocupvisionfeature.hpp"
#include "vision_interfaces/msg/pan_tilt_msgs.hpp"
#include "vision_interfaces/msg/pan_angle_compensation.hpp"
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"

#include "robocup_localization/msg/robocup_localization.hpp"
#include "robocup_localization/msg/technical_challenge.hpp"
#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/master2localization.hpp"

#include "../robot_odom_model/robot_odom_model.hpp"
#include "../objects/obstacle.hpp"
#include "../objects/ball.hpp"
#include "../objects/line_feature.hpp"
#include "../technical_challenge/colaborate_localization.hpp"

#define DEG2RAD (M_PI / 180)

// ===== Subscribe =====
struct VisionMsg
{
  int Ball_cam_X = 0;
  int Ball_cam_Y = 0;
  float Ball_2d_X = 0.0f;
  float Ball_2d_Y = 0.0f;
  float Ball_D = 0.0f;
  float PAN = 0.0f;
  float TILT = 0.0f;
  float Ball_speed_X = 0.0f;
  float Ball_speed_Y = 0.0f;
  vector<float> ROBOT_VEC_X;
  vector<float> ROBOT_VEC_Y;
  int Ball_speed_level = 0;
  int Scan_mode = 0;
  int flag_pan = 0;
};

struct ImuMsg
{
  float yaw = 0.0f;
};

struct IkMsg
{
  int x = 0;
  int y = 0;
};

struct UDPMsg
{
  int robotnum = 0;
  int robotcase = 0;
  int localx = 0;
  int localy = 0;
  int localyaw = 0;
  int ballx = 0;
  int bally = 0;
};

struct MasterMsg
{
  int master_target_x = 0;
  int master_target_y = 0;
  float master_best_position_attack_x = 0.0f;
  float master_best_position_attack_y = 0.0f;
  float master_best_position_defend_x = 0.0f;
  float master_best_position_defend_y = 0.0f;
  float master_angle_to_target = 0.0f;
  std::string master_move2ball_phase = "";
};

struct GameMsg
{
  int mySide = 1;
  int penalty = 0;
  int position = 0;
  int robotNum = 0;
  int state = 0;
};

struct PancompMsg
{
  float target_yaw_deg = 0.0f;
};

struct TechnicalChallengeMsg
{
  int ball_in_approach = 0;
  double blind_robot_x = 0.0;
  double blind_robot_y = 0.0;
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
  rclcpp::Publisher<robocup_localization::msg::TechnicalChallenge>::SharedPtr technical_challenge_pub_;

  void publish_localization(
      float ball_x, float ball_y,
      float ball_speed_x, float ball_speed_y,
      float robot_x, float robot_y,
      const std::vector<float> &obstacles_x,
      const std::vector<float> &obstacles_y);
  void publish_pan_comp();
  void publish_technical_challenge();
  void getParams();

  string robot_feature;
  string imu_topic;
  string vision_topic;
  string vision_feature_topic;
  string ik_topic;
  string game_control_topic;
  string udp_topic;
  string master_topic;
  string local_pub_topic;

  int vision_data_cnt = 0;
  int vision_data_size = 0;
  bool valid_observation_ = false;

  // callback
  int vision_callback_timer = 0;

  // For print
  int robot_sight_flag = 0;

  // For UI
  int set_ball_flag = 0;
  int set_obstacle_flag = 0;
  std::vector<OBSTACLE> debug_obstacles;

  // For Objects
  VisionMsg visionMSG;
  ImuMsg imuMSG;
  IkMsg ikMSG;
  UDPMsg udpMSG;
  MasterMsg masterMSG;
  PancompMsg panCompMSG;
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
  std::vector<OBSTACLE> obstacles;

  // For gamecontrol
  GameMsg gameMSG;
  TechnicalChallengeMsg technical_challengeMSG;

  // For particle
  int particle_num = 8000;
  double weight_update_threshold = 14.0;
  float particle_range = 0.0;
  std::vector<ROBOT> pt;

  // For settings
  bool soccer_mode_ = true;
  double foot_align_odom_scale_ = 0.3;
  bool corner_kick_receiver_ = false;
  bool local_helper_ = false;
  bool local_kicker_ = false;
  bool high_kicker_ = false;
  bool noise_enable_ = true;
  int local_helper_dir = 0; // 0: left, 1: right

  // Thread safety: vision_point_vect 공유 보호
  std::mutex vision_mutex;
  // Thread safety: robot0 공유 보호 (ROS 콜백 스레드 , Qt 메인 스레드)
  std::mutex robot0_mutex;
  // Thread safety: pt[] 공유 보호 (ikCallback vs mclProcessing)
  std::mutex pt_mutex;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(const vision_interfaces::msg::Robocupvision::SharedPtr msg);
  void visionFeatureCallback(const vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg);
  void panTiltCallback(const vision_interfaces::msg::PanTiltMsgs::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg);
  void blindRobotCallback(const robocup_localization::msg::TechnicalChallenge::SharedPtr msg);
  void yaw_compensation();

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvision>::SharedPtr vision_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvisionfeature>::SharedPtr vision_feature_sub_;
  rclcpp::Subscription<vision_interfaces::msg::PanTiltMsgs>::SharedPtr pan_tilt_sub_;
  bool camera_stable_ = false;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<gamecontroller::msg::Gamecontroldata>::SharedPtr game_control_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2localization>::SharedPtr master_sub_;
  rclcpp::Subscription<robocup_localization::msg::TechnicalChallenge>::SharedPtr blind_robot_sub_;

  rclcpp::Publisher<vision_interfaces::msg::PanAngleCompensation>::SharedPtr pan_angle_comp_pub_;

  int yaw_correction_cooldown_ = 100;
  int imu_ignore_count_ = 0;
  int loop_hz = 20;
  std::vector<double> goal_post_pan_deg_buf_;

  static constexpr double YAW_DRIFT_THRESHOLD_DEG = 60.0;

  // ikCallback 작업 변수
  double ik_Xmoved, ik_Ymoved;
  double ik_prev_x, ik_prev_y;
  double ik_dx, ik_dy;
  double ik_rot1, ik_trans;
  double ik_rot1_dev, ik_trans_dev;
  double ik_d_yaw = 0.0; // IMU yaw 변화량 (deg), 파티클 θ 업데이트에 사용

  // rot1/trans/rot2 모션 모델 alpha 계수 (튜닝 가능)
  // alpha1: rot 노이즈 ← rot 크기
  // alpha2: rot 노이즈 ← trans 크기
  // alpha3: trans 노이즈 ← trans 크기
  // alpha4: trans 노이즈 ← rot 크기
  // alpha5: yaw 노이즈 ← yaw delta 크기
  static constexpr double ALPHA1 = 0.01;
  static constexpr double ALPHA2 = 0.001;
  static constexpr double ALPHA3 = 0.03;
  static constexpr double ALPHA4 = 0.01;
  static constexpr double ALPHA5 = 0.02;
  static constexpr double IK_PX_MIN_X = FieldCoord::toPixelX(-FieldCoord::A / 2);
  static constexpr double IK_PX_MAX_X = FieldCoord::toPixelX(FieldCoord::A / 2);
  static constexpr double IK_PX_MIN_Y = FieldCoord::toPixelY(FieldCoord::B / 2);
  static constexpr double IK_PX_MAX_Y = FieldCoord::toPixelY(-FieldCoord::B / 2);

  //   1단계: ALPHA3부터 (위치 노이즈 제일 체감)
  //        로봇 직진 → 파티클 퍼짐이 실제 오차와 비슷하면 OK

  // 2단계: ALPHA1 (방향 노이즈)
  //        제자리 회전 → 랜드마크 매칭 후 수렴 속도 확인

  // 3단계: ALPHA5 (yaw 노이즈)
  //        IMU 드리프트 구간에서 파티클 θ가 랜드마크로 보정되는지 확인

  // 4단계: ALPHA2, ALPHA4는 마지막에 미세 조정

  void startMclProcessing();
  void mclProcessing();

  std::thread mcl_thread_;
  std::condition_variable mcl_cv_;
  std::mutex mcl_mutex_;
  bool mcl_pending_ = false;
  bool mcl_running_ = false;

Q_SIGNALS:
  void rosShutDown();
};

#endif
