#pragma once

#include <algorithm>
#include <vector>

#include "vision_interfaces/msg/robocupvisionfeature.hpp"

#include "mlp_localization/mlp_onnx_infer.hpp"

// vision_feature 메시지 → MLP 모델 입력 차원 맞추기
// ============================================================
// 모델은 타입별로 고정 슬롯 수를 기대한다 (corner 6 / t 4 / cross 2 / goalpost 2).
// 여기서는 메시지의 병렬 배열(x[], y[])을 (x, y) 쌍 목록으로 바꾸는 것까지만
// 하고, 슬롯 채우기·거리순 정렬·FOV 게이팅은 OnnxLocalizer::buildObs 가 맡는다.
// 그쪽이 학습 규약(dataset_constants.py)과 짝을 맞춰야 하는 부분이라서다.
//
// MCL 경로의 VisionProcessing 과 대응된다 — 같은 메시지를 각자 필요한 형태로
// 푼다. MCL 은 world-frame 픽셀이 필요하고, MLP 는 로봇 기준 mm 원본을 쓴다.
namespace MlpDimensionAdapter
{

// 모델 입력으로 넘길 타입별 특징점 묶음.
struct Features
{
  std::vector<learned_loc::FeaturePoint> corner;
  std::vector<learned_loc::FeaturePoint> t_shape;
  std::vector<learned_loc::FeaturePoint> cross;
  std::vector<learned_loc::FeaturePoint> goalpost;  // 소스 없음 → 항상 빈 값
};

/// 병렬 배열 두 개를 (x, y) 쌍 목록으로. 길이가 다르면 짧은 쪽에 맞춘다.
inline std::vector<learned_loc::FeaturePoint> pack(const std::vector<double> &vx,
                                                   const std::vector<double> &vy)
{
  std::vector<learned_loc::FeaturePoint> out;
  const size_t n = std::min(vx.size(), vy.size());
  out.reserve(n);
  for (size_t i = 0; i < n; ++i)
    out.push_back({vx[i], vy[i]});
  return out;
}

/// 메시지 한 프레임에서 타입별 특징점을 뽑는다.
/// goalpost 는 비전이 좌표를 주지 않아 비워둔다 → obs 슬롯이 valid=0 으로 채워진다.
inline Features fromMessage(
    const vision_interfaces::msg::Robocupvisionfeature &msg)
{
  Features f;
  f.corner = pack(msg.corner_point_vec_x, msg.corner_point_vec_y);
  f.t_shape = pack(msg.t_point_vec_x, msg.t_point_vec_y);
  f.cross = pack(msg.cross_point_vec_x, msg.cross_point_vec_y);
  return f;
}

}  // namespace MlpDimensionAdapter
