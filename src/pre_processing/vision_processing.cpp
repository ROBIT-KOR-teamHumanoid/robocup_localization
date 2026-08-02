#include "pre_processing/vision_processing.hpp"

#include <cmath>

#include "field_profile/field_coordinate.hpp"

namespace VisionProcessing
{

PointPx toWorldOffsetPx(double vx_mm, double vy_mm, double yaw_deg)
{
  const double yaw_rad = yaw_deg * M_PI / 180.0;
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);

  const double x_px = vx_mm * FieldCoord::MM2PX;
  const double y_px = vy_mm * FieldCoord::MM2PX;

  return PointPx{(-1) * (x_px * c + y_px * s), (x_px * s - y_px * c)};
}

PointPx toFieldPx(double vx_mm, double vy_mm, const Pose &pose)
{
  const PointPx off = toWorldOffsetPx(vx_mm, vy_mm, pose.yaw_deg);
  return PointPx{pose.x_px + off.x, pose.y_px + off.y};
}

int append(std::vector<VISION_POINT> &out,
           const std::vector<double> &conf,
           const std::vector<double> &dist,
           const std::vector<double> &vx_mm,
           const std::vector<double> &vy_mm,
           const std::vector<double> &ball_angle,
           const std::vector<double> &ball_dist,
           int feature_type,
           const Pose &pose)
{
  const int n = static_cast<int>(conf.size());
  if (n == 0)
    return 0;

  out.reserve(out.size() + n);

  for (int i = 0; i < n; i++)
  {
    VISION_POINT vp;

    vp.CONFIDENCE = conf[i];
    vp.DISTANCE = dist[i];

    // 원본 상대 좌표 (mm). 파티클별 재변환에 쓰이므로 손대지 않는다.
    vp.RAW_X_MM = vx_mm[i];
    vp.RAW_Y_MM = vy_mm[i];

    // 로봇 기준 mm -> world-frame 맵 픽셀 오프셋 (위치가 아니라 벡터)
    const PointPx off = toWorldOffsetPx(vx_mm[i], vy_mm[i], pose.yaw_deg);
    vp.POINT_VEC_X = off.x;
    vp.POINT_VEC_Y = off.y;

    // 관측 시점의 로봇 위치. worldX/worldY 가 이걸 더해 절대 좌표를 만든다.
    vp.STD_X = pose.x_px;
    vp.STD_Y = pose.y_px;

    vp.FEATURE_TYPE = feature_type;

    // 비전이 항상 채워주지 않으므로 길이를 확인하고 없으면 0.
    vp.BALL_RELATIVE_ANGLE =
        (i < static_cast<int>(ball_angle.size())) ? ball_angle[i] : 0.0;
    vp.BALL_FEATURE_DISTANCE =
        (i < static_cast<int>(ball_dist.size())) ? ball_dist[i] : 0.0;

    out.push_back(vp);
  }

  return n;
}

}  // namespace VisionProcessing
