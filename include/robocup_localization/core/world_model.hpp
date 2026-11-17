#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "robocup_localization/core/types/ball.hpp"
#include "robocup_localization/core/types/obstacle.hpp"
#include "robocup_localization/core/types/pose2d.hpp"

namespace robocup_localization::core
{

  struct VisionObservation
  {
    bool ball_detected = false;

    float ball_x_mm = 0.0f;
    float ball_y_mm = 0.0f;
    float ball_distance_mm = 0.0f;

    float ball_speed_x = 0.0f;
    float ball_speed_y = 0.0f;
    int ball_prediction_level = 0;

    std::vector<float> obstacle_x_mm;
    std::vector<float> obstacle_y_mm;
  };

  struct TeammateObservation
  {
    bool active = false;

    Pose2D pose;
    int state = 0;

    float ball_x = 0.0f;
    float ball_y = 0.0f;
  };

  struct WorldModelConfig
  {
    double teammate_timeout_sec = 10.0;
    double local_ball_lost_sec = 3.0;
    double teammate_ball_takeover_sec = 0.5;
    double obstacle_timeout_sec = 15.0;
  };

  struct WorldModelSnapshot
  {
    BALL ball;
    bool manual_ball_active = false;
    int ball_prediction_level = 0;
    std::vector<OBSTACLE> obstacles;
    std::vector<OBSTACLE> manual_obstacles;
    std::array<TeammateObservation, 4> teammates;
  };

  class WorldModel
  {
  public:
    explicit WorldModel(WorldModelConfig config = {});

    void updateVision(
        const Pose2D &robot_pose,
        const VisionObservation &observation);

    void updateTeammate(
        std::size_t index,
        const TeammateObservation &observation);

    void setManualBall(float x, float y);
    void clearManualBall();
    void clearTeammates();
    void addManualObstacle(float x, float y);
    void removeLastManualObstacle();

    void update(double dt_seconds);

    const WorldModelSnapshot &snapshot() const noexcept;

  private:
    BALL local_ball_;
    double local_ball_age_seconds_ = 0.0;
    bool local_ball_valid_ = false;
    bool local_ball_observed_ = false;

    BALL manual_ball_;
    bool manual_ball_active_ = false;

    void expireTeammates(double dt_seconds);
    void expireObstacles(double dt_seconds);
    void selectBall(double dt_seconds);
    const TeammateObservation *closestTeammateWithBall() const;

    std::array<double, 4> teammate_age_seconds_{};
    std::vector<double> obstacle_age_seconds_;

    WorldModelConfig config_;
    WorldModelSnapshot snapshot_;
  };

}
