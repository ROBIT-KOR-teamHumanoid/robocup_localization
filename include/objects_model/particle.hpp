#pragma once

#include "objects_model/pose2d.hpp"

// MCL 파티클 — pose 가설 하나
// ============================================================
// 예전엔 ROBOT 을 그대로 파티클로 썼는데, 파티클에겐 의미 없는 필드
// (자취 vector, odom 캘리브레이션, 팀 신선도, UI 표시값)가 딸려와서
// 8000개분이 낭비였다. 매 프레임 스냅샷으로 복사되는 자료라 크기가 곧 비용이다.
//
// 파티클은 pose 그 자체다. 가중치는 MEASUREMENT 가 인덱스로 따로 든다.
using Particle = Pose2D;
