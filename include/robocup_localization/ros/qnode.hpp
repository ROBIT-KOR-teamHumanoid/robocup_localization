#ifndef robocup_localization_QNODE_HPP_
#define robocup_localization_QNODE_HPP_

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
#include "robocup_localization/app/kidnap_recovery.hpp"
#include "robocup_localization/inference/observation_encoder.hpp"
#include "robocup_localization/inference/onnx_localizer.hpp"

#define DEG2RAD (M_PI / 180)

// ===== Subscribe =====
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

  // 학습형(ONNX) 추론. vision_feature 콜백이 프레임을 넘겨 주면 전용 스레드가
  // 돌린다 (mcl_thread_ 와 같은 구조). 결과는 inference_state_ 에 모았다가
  // localizationSnapshot() 으로만 밖에 나간다.
  robocup_localization::inference::InferenceConfig inference_config_;
  robocup_localization::inference::ObservationEncoder inference_encoder_;
  robocup_localization::inference::OnnxLocalizer inference_;
  mutable std::mutex inference_mutex_;   // inference_state_ (결과) 보호
  robocup_localization::app::InferenceStateSnapshot inference_state_;

  // 추론 스레드에 넘길 한 프레임. 슬롯이 하나뿐이라 새 프레임이 오면 아직
  // 처리 못 한 이전 프레임을 덮어쓴다 — 오래된 자세 추정은 쓸모가 없으므로
  // 밀리는 것보다 버리는 편이 낫다.
  struct InferenceJob
  {
    vision_interfaces::msg::Robocupvisionfeature::SharedPtr msg;
    double yaw_deg = 0.0;
    bool has_features = false;
  };

  void initInference();
  void startInference();
  void inferenceProcessing();
  void runInference(const InferenceJob& job);
  std::string resolveModelPath(const std::string& path) const;

  // 납치 판정. 추론 스레드에서만 만지므로 별도 락이 필요 없다.
  robocup_localization::app::KidnapRecovery kidnap_;
  void applyKidnapVerdict(const robocup_localization::app::KidnapVerdict& verdict);

  std::thread inference_thread_;
  std::condition_variable inference_cv_;
  std::mutex inference_job_mutex_;   // 아래 4개 보호
  InferenceJob inference_job_;
  bool inference_pending_ = false;
  bool inference_running_ = false;
  unsigned long inference_dropped_ = 0;

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

  double ik_d_yaw = 0.0; // IMU yaw 변화량 (deg), 파티클 θ 업데이트에 사용

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
