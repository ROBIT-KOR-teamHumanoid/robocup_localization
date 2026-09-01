#pragma once

#include <cstddef>
#include <vector>

namespace robocup_localization::app
{

struct KidnapRecoveryConfig
{
  /// 이 클래스를 아예 끄는 스위치.
  bool enable = false;

  /// PF 와 MLP 가 이만큼 떨어져야 후보로 센다 (m).
  /// MclTune::MAX_JUMP_M 보다 커야 한다 — 그 아래는 PF 가 정상적으로
  /// 따라가는 중인 구간이라 오검출이 된다.
  double disagree_dist_m = 2.0;

  /// 누적된 MLP 출력들이 모여 있어야 하는 반경 (m).
  double cluster_radius_m = 0.5;

  /// 판정에 필요한 최소 프레임 수.
  int min_frames = 5;

  /// 발화 후 재발화 금지 시간 (s).
  double cooldown_sec = 3.0;
};

struct KidnapVerdict
{
  /// 납치로 판정했는가. 참이면 아래 좌표로 자세를 옮긴다.
  bool kidnapped = false;

  /// 옮길 좌표 — 누적된 MLP 출력들의 무게중심 (m).
  double x_m = 0.0;
  double y_m = 0.0;

  /// 판정에 쓰인 프레임 수 / 그 프레임들의 클러스터 반경 (m). 로그용.
  int frames = 0;
  double spread_m = 0.0;
};

class KidnapRecovery
{
public:
  void configure(const KidnapRecoveryConfig& config);
  const KidnapRecoveryConfig& config() const noexcept { return config_; }

  KidnapVerdict update(
      double now_sec,
      double pf_x_m, double pf_y_m,
      bool mlp_valid,
      double mlp_x_m, double mlp_y_m);

  /// 누적을 버린다. 사람이 자세를 직접 지정했을 때처럼 이력이 무의미해지면 부른다.
  void reset();

  /// 지금까지 쌓인 프레임 수 (디버깅용).
  std::size_t pending() const noexcept { return samples_.size(); }

private:
  struct Sample
  {
    double x_m = 0.0;
    double y_m = 0.0;
  };

  /// 누적 샘플의 무게중심.
  Sample centroid() const;

  /// 무게중심에서 가장 먼 샘플까지의 거리 (m).
  double spread() const;

  KidnapRecoveryConfig config_;
  std::vector<Sample> samples_;
  double cooldown_until_sec_ = 0.0;
  bool cooldown_active_ = false;
};

}  // namespace robocup_localization::app
