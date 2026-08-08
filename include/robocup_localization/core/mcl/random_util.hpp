#pragma once

#include "robocup_localization/core/types/particle.hpp"

namespace robocup_localization::core
{

    // 파티클 재살포 시 θ 초기 분산 (deg). 너무 작으면 theta 다양성 소실.
    inline constexpr double THETA_INIT_SPREAD_DEG = 0;

    /// 평균 average, 표준편차 stdev 인 가우시안 난수
    double gaussianRandom(double average, double stdev);

    /// [0, upper) 균등 난수. systematic resampling 시작 오프셋에 쓴다.
    double uniformRandom(double upper);

    /// 파티클 하나를 (x, y) 주변 particle_range 반경으로 재살포
    void randomizeParticle(
        Particle &particle, int x, int y, float particle_range, float theta);

} // namespace robocup_localization::core
