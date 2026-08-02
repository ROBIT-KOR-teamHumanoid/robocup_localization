#pragma once

#include <vector>

#include "objects_model/particle.hpp"

// 파티클 집합 연산
// ============================================================
// 파티클 전체를 한꺼번에 손대는 동작들. 모션 전파(MotionModel)나 가중치
// 계산(MclEngine)과 달리, 외부 사건(반칙 리셋, 골대 y부호 불일치)으로
// 파티클을 통째로 옮기거나 뒤집는 경우다.
//
// 락은 호출자가 pt_mutex 를 잡은 상태에서 부를 것.
namespace ParticleReset
{

/// y=0 축(골대끼리 잇는 장축) 기준으로 파티클을 미러링한다.
/// 골대 두 개가 모두 y=0 위에 있어 랜드마크만으로는 로봇이 위쪽 터치라인
/// 근처인지 아래쪽인지 구분할 수 없다. 골대 pan 으로 판정한 y 부호
/// (YawCompensator::computeYSign)가 현재 추정과 반대이면 통째로 뒤집는다.
/// 위치만 뒤집으면 헤딩이 관측과 어긋나므로 yaw 도 180-yaw 로 함께 뒤집는다.
void mirrorAboutMidline(std::vector<Particle> &pt, int particle_num);

/// 지정한 pose 주변으로 파티클을 다시 뿌린다 (가우시안, sigma = range_px).
/// 반칙 후 복귀처럼 위치가 외부에서 확정될 때 쓴다.
void scatterAround(std::vector<Particle> &pt, int particle_num,
                   float x_px, float y_px, float yaw_deg, float range_px);

}  // namespace ParticleReset
