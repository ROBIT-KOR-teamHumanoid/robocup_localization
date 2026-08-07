#include "monte_carlo_localization/mcl_node.hpp"

#include "monte_carlo_localization/particle_reset.hpp"

MclNode::MclNode() {}

MclNode::~MclNode()
{
  stop();
}

void MclNode::start()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
      return;
    running_ = true;
  }
  thread_ = std::thread(&MclNode::loop, this);
}

void MclNode::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_one();

  if (thread_.joinable())
    thread_.join();
}

MclNode::Snapshot MclNode::snapshot()
{
  Snapshot s;
  // 락 순서 pt -> robot0
  {
    std::lock_guard<std::mutex> pt_lock(pt_mutex);
    s.particles = pt;
  }
  {
    std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
    s.robot = robot0;
  }
  {
    // 관측 목록과 랜드마크는 featureCalc 이 vision_mutex 아래에서 갱신한다.
    std::lock_guard<std::mutex> vision_lock(vision_mutex);
    s.observations = Likelihood.vision_point_vect;
    for (int i = 0; i < FieldCoord::FEATURE_POINT_COUNT; i++)
    {
      s.landmark_x[i] = Likelihood.filter.Local_point_x[i];
      s.landmark_y[i] = Likelihood.filter.Local_point_y[i];
      s.landmark_on[i] = Likelihood.filter.Local_point_on_off[i];
    }
    s.circle_center = Likelihood.filter.CIRCLE_CENTER;
    s.circle_r = Likelihood.filter.CIRCLE_R;
  }
  return s;
}

Pose2D MclNode::pose()
{
  std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
  return static_cast<Pose2D>(robot0);
}

void MclNode::placeRobot(float x_px, float y_px, float scatter_range_px)
{
  // 락 순서 pt -> robot0
  std::lock_guard<std::mutex> pt_lock(pt_mutex);
  std::lock_guard<std::mutex> robot0_lock(robot0_mutex);

  robot0.x = x_px;
  robot0.y = y_px;
  robot0.trail.clear();

  ParticleReset::scatterAround(pt, particle_num, robot0.x, robot0.y,
                               robot0.yaw_deg, scatter_range_px);
}

void MclNode::moveRobot(float x_px, float y_px)
{
  std::lock_guard<std::mutex> robot0_lock(robot0_mutex);
  robot0.x = x_px;
  robot0.y = y_px;
  robot0.trail.clear();
}

void MclNode::clearObservations()
{
  std::lock_guard<std::mutex> vision_lock(vision_mutex);
  Likelihood.vision_point_vect.clear();
}

void MclNode::notifyObservation()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = true;
  }
  cv_.notify_one();
}

void MclNode::loop()
{
  // running_ / pending_ 는 반드시 mutex_ 아래에서만 읽는다.
  // 바깥 while 에서 락 없이 running_ 을 읽으면 stop() 의 쓰기와 데이터 경합이다.
  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return pending_ || !running_; });
      if (!running_)
        break;
      pending_ = false;
    }

    // 여기서는 락을 하나도 들고 있지 않다. featureCalc 이 표준 순서대로
    // vision -> pt -> robot0 를 직접 잡는다 (mcl_node.hpp 락 순서 규약 참조).
    Likelihood.featureCalc(*this);
  }
}
