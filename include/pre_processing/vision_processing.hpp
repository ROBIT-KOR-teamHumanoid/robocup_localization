#pragma once

#include <vector>

#include "objects_model/vision_point.hpp"

// vision_feature 메시지 → MCL 관측(VISION_POINT) 변환
// ============================================================
// 비전은 특징점을 **로봇 기준 상대 mm** 로 준다. MCL 은 절대 맵 픽셀에서
// 랜드마크와 비교하므로, 관측 시점의 로봇 pose 를 함께 실어 world-frame 으로
// 올려둔다. 원본 상대값(RAW_*_MM)도 그대로 보존한다 — weight_calculate 가
// 파티클마다 각자의 θ 로 다시 회전해 쓰기 때문이다.
//
// corner / t / cross 세 타입이 필드 이름만 다르고 변환은 동일해서, 한 함수를
// 세 번 부르는 형태로 둔다.
namespace VisionProcessing
{

// 관측 시점의 로봇 pose (맵 픽셀 + 도)
struct Pose
{
  double x_px = 0.0;
  double y_px = 0.0;
  double yaw_deg = 0.0;
};

// 맵 픽셀 좌표 한 점
struct PointPx
{
  double x = 0.0;
  double y = 0.0;
};

/// 로봇 기준 상대 mm 벡터 → world-frame 맵 픽셀 **오프셋**.
/// 위치가 아니라 벡터다. 절대 좌표가 필요하면 toFieldPx 를 쓸 것.
/// 특징점·공·장애물이 모두 같은 규약(비전이 주는 mm)이라 이 한 식을 공유한다.
PointPx toWorldOffsetPx(double vx_mm, double vy_mm, double yaw_deg);

/// 로봇 기준 상대 mm → world-frame 맵 픽셀 **절대 위치**.
PointPx toFieldPx(double vx_mm, double vy_mm, const Pose &pose);

/// 특징점 한 타입을 out 뒤에 덧붙인다. 반환값은 추가된 개수.
///
/// conf/dist/vx/vy 는 같은 길이여야 하며, 길이는 conf 기준으로 잡는다.
/// ball_angle / ball_dist 는 길이가 모자라면 0 으로 채운다(비전이 항상
/// 채워주지는 않는다).
/// feature_type: 0=L(corner) 1=T 2=X(cross)
int append(std::vector<VISION_POINT> &out,
           const std::vector<double> &conf,
           const std::vector<double> &dist,
           const std::vector<double> &vx_mm,
           const std::vector<double> &vy_mm,
           const std::vector<double> &ball_angle,
           const std::vector<double> &ball_dist,
           int feature_type,
           const Pose &pose);

}  // namespace VisionProcessing
