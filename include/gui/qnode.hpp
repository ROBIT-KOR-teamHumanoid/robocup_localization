
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
#include "vision_interfaces/msg/pan_angle_compensation.hpp"
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"

#include "robocup_localization/msg/robocup_localization.hpp"
#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/master2localization.hpp"

#include "objects_model/self_robot.hpp"
#include "objects_model/teammate.hpp"
#include "objects_model/obstacle.hpp"
#include "objects_model/ball.hpp"
#include "monte_carlo_localization/mcl_engine.hpp"
#include "monte_carlo_localization/motion_model.hpp"
#include "monte_carlo_localization/mcl_node.hpp"
#include "pre_processing/ball_tracker.hpp"
#include "mlp_localization/mlp_onnx_infer.hpp"

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
  int ready_count = 0;
};

struct PancompMsg
{
  float target_yaw_deg = 0.0f;
};

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  // rclcpp::init() 은 main() 이 먼저 부른다 — GUI 없이도 뜰 수 있어야 하므로
  // ROS 초기화가 Qt 객체 생성에 묶여 있으면 안 된다.
  QNode();
  ~QNode();

  // ball / obstacles 는 아직 GUI 스레드만 만지므로 이 함수도 거기서 부른다.
  // ROS 타이머로 옮기려면 그 소유권부터 정리해야 한다.
  // 추적 갱신 + 발행. ROS 타이머가 돌리므로 GUI 가 멈춰도 계속된다.
  void localizationTick();
  void publish_localization();
  void getParams();

  // 추정 로직 + 상태 소유자. 아래 참조 별명들보다 먼저 선언돼야 한다.
  MclNode mcl_;

  // 공·장애물 추적기. 아래 별명들보다 먼저 선언돼야 한다.
  BallTracker tracker_;

  string robot_feature;
  string imu_topic;
  string vision_topic;
  string vision_feature_topic;
  string ik_topic;
  string game_control_topic;
  string udp_topic;
  string master_topic;
  string local_pub_topic;


  // callback
  int vision_callback_timer = 0;

  // For print
  int robot_sight_flag = 0;

  // For UI
  int set_ball_flag = 0;
  int set_obstacle_flag = 0;

  // For Objects
  VisionMsg visionMSG;
  ImuMsg imuMSG;
  IkMsg ikMSG;
  UDPMsg udpMSG;
  MasterMsg masterMSG;
  PancompMsg panCompMSG;
  // 팀 로봇 pose. udpCallback(ROS 스레드)이 쓰고 화면(Qt)이 읽으므로
  // 직접 읽지 말고 teammates() 로 한 번에 복사할 것.
  Teammate robot1, robot2, robot3, robot4;
  std::mutex teammate_mutex_;

  struct TeamSnapshot
  {
    Teammate r[4];
  };
  TeamSnapshot teammates();
  void resetTeammates();

  // masterMSG 는 masterCallback(ROS)이 쓰고 화면(Qt)이 읽는다.
  // master_move2ball_phase 가 std::string 이라 직접 읽으면 찢어진 값이
  // 아니라 크래시가 난다. 반드시 아래 두 함수로만 접근할 것.
  std::mutex master_mutex_;
  MasterMsg masterState();
  void clearMasterTarget();

  // For gamecontrol
  GameMsg gameMSG;


  // For settings
  bool soccer_mode_ = true;
  double foot_align_odom_scale_ = 0.3;

  // 적용된 필드 규격 ("S" | "M" | "L")
  std::string field_size_ = "L";

  // ===== Learned (ONNX) localization =====
  bool learned_mlp_enable_ = false;
  bool learned_reloc_enable_ = false;
  std::string learned_mlp_model_;
  std::string learned_reloc_model_;
  int learned_reloc_topk_ = 30;
  double learned_sign_right_ = -1.0;
  double learned_sign_forward_ = 1.0;

  learned_loc::OnnxLocalizer learned_;
  std::mutex learned_mutex_;                    // 결과 공유 보호 (ROS vs Qt)
  learned_loc::MlpResult learned_mlp_result_;   // 필드 프레임(m)
  learned_loc::RelocResult learned_reloc_result_;

  // 추정 상태는 MclNode 가 소유한다. 아래는 이름을 유지하기 위한 참조 별명이라
  // 값을 따로 들고 있지 않다 — 호출부(main_window 89곳, line_feature 12곳)를
  // 건드리지 않고 소유권만 옮기기 위한 중간 단계다.
  // 락 순서 규약(vision → pt → robot0)은 mcl_node.hpp 참조.
  // 공·장애물은 BallTracker 가 소유한다 (아래는 이름 유지를 위한 별명).
  BALL &ball = tracker_.ball;
  BALL &ball1 = tracker_.ball1;
  BALL &ball2 = tracker_.ball2;
  BALL &ball3 = tracker_.ball3;
  BALL &ball4 = tracker_.ball4;
  std::vector<OBSTACLE> &obstacles = tracker_.obstacles;
  std::vector<OBSTACLE> &debug_obstacles = tracker_.debug_obstacles;

  SelfRobot &robot0 = mcl_.robot0;
  std::vector<Particle> &pt = mcl_.pt;
  int &particle_num = mcl_.particle_num;
  float &particle_range = mcl_.particle_range;
  MclEngine &Likelihood = mcl_.Likelihood;
  MotionModel &motion_ = mcl_.motion;
  double &weight_update_threshold = mcl_.weight_update_threshold;
  bool &noise_enable_ = mcl_.noise_enable;
  bool &valid_observation_ = mcl_.valid_observation;
  int &vision_data_cnt = mcl_.vision_data_cnt;
  int &vision_data_size = mcl_.vision_data_size;
  std::mutex &vision_mutex = mcl_.vision_mutex;
  std::mutex &pt_mutex = mcl_.pt_mutex;
  std::mutex &robot0_mutex = mcl_.robot0_mutex;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(const vision_interfaces::msg::Robocupvision::SharedPtr msg);
  void visionFeatureCallback(const vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg);
  void yaw_compensation();
  void publish_pan_comp();  // yaw_compensation 내부에서만 사용
  void runLearnedInference(
      const vision_interfaces::msg::Robocupvisionfeature::SharedPtr &msg,
      double yaw_deg);

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvision>::SharedPtr vision_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvisionfeature>::SharedPtr vision_feature_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<gamecontroller::msg::Gamecontroldata>::SharedPtr game_control_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2localization>::SharedPtr master_sub_;

  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::Publisher<robocup_localization::msg::RobocupLocalization>::SharedPtr localization_pub_;
  rclcpp::Publisher<vision_interfaces::msg::PanAngleCompensation>::SharedPtr pan_angle_comp_pub_;

  int yaw_correction_cooldown_ = 100;
  int imu_ignore_count_ = 0;
  int loop_hz = 20;
  std::vector<double> goal_post_pan_deg_buf_;

  static constexpr double YAW_DRIFT_THRESHOLD_DEG = 60.0;

  // ikCallback 작업 변수
  double ik_d_yaw = 0.0; // IMU yaw 변화량 (deg), 파티클 θ 업데이트에 사용

  // 모션 모델 (alpha 계수·클램프는 MotionModel 이 소유)

  // MCL 워커 스레드. 상태(robot0/pt/Likelihood)는 아직 QNode 소유라
  // owner 참조로 접근한다.

Q_SIGNALS:
  void rosShutDown();
};

#endif
