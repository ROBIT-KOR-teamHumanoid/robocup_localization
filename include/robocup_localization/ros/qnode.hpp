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
#include "humanoid_interfaces/msg/ik_coord_msg.hpp"

#include "robocup_localization/msg/robocup_localization.hpp"
#include "gamecontroller/msg/gamecontroldata.hpp"
#include "humanoid_interfaces/msg/udp2master.hpp"
#include "humanoid_interfaces/msg/master2localization.hpp"

#include "robocup_localization/core/mcl/robot_odom_model.hpp"
#include "robocup_localization/core/types/obstacle.hpp"
#include "robocup_localization/app/localization_command.hpp"
#include "robocup_localization/app/localization_config.hpp"
#include "robocup_localization/app/localization_engine.hpp"
#include "robocup_localization/app/localization_snapshot.hpp"

#define DEG2RAD (M_PI / 180)

// ===== Subscribe =====
/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread,
              public robocup_localization::app::LocalizationCommand,
              public robocup_localization::app::LocalizationSnapshotSource
{
  Q_OBJECT
public:
  QNode(int argc, char **argv);
  ~QNode() override;

  robocup_localization::app::LocalizationSnapshot localizationSnapshot() const override;
  robocup_localization::app::LocalizationConfig localizationConfig() const noexcept override;
  void previewRobotPosition(float x, float y) override;
  void placeRobot(float x, float y, float particle_range) override;
  void setRobotOdometryCalibration(
      float forward, float backward, float left, float right) override;
  void setManualBall(float x, float y) override;
  void clearManualBall() override;
  void clearTeammates() override;
  void addManualObstacle(float x, float y) override;
  void removeLastManualObstacle() override;
  void setParticleRange(float range) override;
  void clearParticleFilterVision() override;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<robocup_localization::msg::RobocupLocalization>::SharedPtr localization_pub_;

  std::string robot_feature;
  std::string imu_topic;
  std::string vision_topic;
  std::string vision_feature_topic;
  std::string ik_topic;
  std::string game_control_topic;
  std::string udp_topic;
  std::string master_topic;
  std::string local_pub_topic;
  bool soccer_mode_ = true;
  double foot_align_odom_scale_ = 0.3;

  robocup_localization::app::LocalizationEngine localization_engine_;
  robocup_localization::app::LocalizationConfig localization_config_;
  float particle_range_ = 0.0f;
  ROBOT robot0;
  mutable std::mutex robot0_mutex;
  robocup_localization::app::MasterStateSnapshot master_state_;
  robocup_localization::app::GameStateSnapshot game_state_;
  mutable std::mutex master_state_mutex_;
  mutable std::mutex game_state_mutex_;
  int vision_pan_flag_ = 0;

  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void visionCallback(const vision_interfaces::msg::Robocupvision::SharedPtr msg);
  void visionFeatureCallback(const vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg);
  void ikCallback(const humanoid_interfaces::msg::IkCoordMsg::SharedPtr msg);
  void gameControlCallback(const gamecontroller::msg::Gamecontroldata::SharedPtr msg);
  void udpCallback(const humanoid_interfaces::msg::Udp2master::SharedPtr msg);
  void masterCallback(const humanoid_interfaces::msg::Master2localization::SharedPtr msg);
  void publish_localization(
      float ball_x, float ball_y,
      float ball_speed_x, float ball_speed_y,
      float robot_x, float robot_y,
      const std::vector<float>& obstacles_x,
      const std::vector<float>& obstacles_y);
  void getParams();
  void publishWorldModelSnapshot(
      const robocup_localization::core::WorldModelSnapshot& snapshot);
  void resetParticles(float x, float y, float yaw_deg, float range);

  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvision>::SharedPtr vision_sub_;
  rclcpp::Subscription<vision_interfaces::msg::Robocupvisionfeature>::SharedPtr vision_feature_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::IkCoordMsg>::SharedPtr ik_sub_;
  rclcpp::Subscription<gamecontroller::msg::Gamecontroldata>::SharedPtr game_control_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Udp2master>::SharedPtr udp_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2localization>::SharedPtr master_sub_;

  std::vector<double> goal_post_pan_deg_buf_;

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
