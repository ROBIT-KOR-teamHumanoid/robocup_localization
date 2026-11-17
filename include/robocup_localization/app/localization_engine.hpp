#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include "robocup_localization/core/mcl/particle_filter.hpp"
#include "robocup_localization/core/world_model.hpp"

namespace robocup_localization::app
{

    class LocalizationEngine
    {
    public:
        explicit LocalizationEngine(core::WorldModelConfig config = {});

        void configureParticleFilter(const core::ParticleFilterConfig &config);
        void setParticleRange(float range);
        void resetParticles(float x, float y, float yaw_deg, float range);
        void clearParticleFilterVision();
        void submitFeatures(
            std::vector<VISION_POINT> observations,
            bool valid_observation,
            double robot_yaw_deg);
        core::MclResult correctParticles(
            float robot_x, float robot_y, float robot_yaw_deg);
        void setBallObservation(double relative_x_mm, double relative_y_mm);
        void mirrorParticlesY(float map_height);
        void setRobotYSign(int sign);
        int robotYSign() const;
        void applyParticleMotion(const core::ParticleMotion &motion);
        core::ParticleFilterSnapshot particleFilterSnapshot() const;

        void updateVision(
            const core::Pose2D &robot_pose,
            const core::VisionObservation &observation);

        void updateTeammate(
            std::size_t index,
            const core::TeammateObservation &observation);

        void setManualBall(float x, float y);
        void clearManualBall();
        void clearTeammates();
        void addManualObstacle(float x, float y);
        void removeLastManualObstacle();

        void update(double dt_seconds);

        core::WorldModelSnapshot snapshot() const;

    private:
        mutable std::mutex mutex_;
        core::WorldModel world_model_;
        core::ParticleFilter particle_filter_;
    };

}
