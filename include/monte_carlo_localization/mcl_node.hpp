#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "monte_carlo_localization/mcl_engine.hpp"
#include "objects_model/particle.hpp"
#include "objects_model/self_robot.hpp"
#include "objects_model/teammate.hpp"
#include "monte_carlo_localization/motion_model.hpp"

// MCL 워커 스레드
// ============================================================
// 관측이 한 프레임 들어올 때마다 깨어나 추정 사이클(가중치 계산 → 리샘플 →
// synthetic 재주입)을 한 번 돈다. ROS 콜백 스레드에서 분리한 이유는 계산이
// 파티클 수(8000)에 비례해 무거워서 spin 루프를 막기 때문이다.
//
// 신호는 "밀린 만큼 쌓지 않는다" — pending 은 bool 이라 콜백이 처리보다 빨라도
// 큐가 자라지 않고 최신 프레임 하나만 처리한다. 위치추정에서는 오래된 관측을
// 뒤늦게 반영하는 것보다 건너뛰는 편이 낫다.
//
// 추정 상태(robot0 / pt / Likelihood)를 이 클래스가 소유한다. QNode 는 ROS
// 배선만 담당하고, MainWindow 와 함께 아직은 참조 별명으로 직접 붙어 있다.
// 그 접근을 좁은 스냅샷 인터페이스로 바꾸는 것이 다음 단계다.
class MclNode
{
public:
  MclNode();
  ~MclNode();

  MclNode(const MclNode &) = delete;
  MclNode &operator=(const MclNode &) = delete;

  /// 워커 스레드 시작. soccer_mode 가 아니면 부르지 않는다.
  void start();

  /// 스레드에 종료를 알리고 join 한다. 소멸자에서도 호출되므로 중복 호출 안전.
  void stop();

  /// 새 관측이 준비됐음을 알린다 (ROS 콜백 스레드에서 호출).
  void notifyObservation();

  bool running() const { return running_; }

  // GUI 표시용 상태 사본.
  // 화면은 여러 값을 순차로 읽는데, 그 사이 추정이 갱신되면 x 와 y 가 서로
  // 다른 시점의 조합이 되어 로봇이 어긋난 자리에 그려진다. 한 번에 복사해
  // 일관된 시점을 보장하고, 락 보유 시간도 그리기 전체가 아니라 복사 동안으로
  // 줄인다.
  struct Snapshot
  {
    SelfRobot robot;
    std::vector<Particle> particles;
  };
  Snapshot snapshot();

  // ── 외부 확정 명령 ─────────────────────────────────────────────
  // 관측이 아니라 "여기 있다"가 밖에서 확실할 때 쓴다 (GUI 드래그, 반칙 복귀).
  // 락은 이 함수들이 표준 순서대로 직접 잡는다.

  /// pose 를 확정하고 파티클을 그 주변에 다시 뿌린다. 자취도 지운다.
  void placeRobot(float x_px, float y_px, float scatter_range_px);

  /// pose 만 옮긴다 (드래그 중 미리보기). 파티클은 건드리지 않는다.
  void moveRobot(float x_px, float y_px);

  // ── 추정 상태 ──────────────────────────────────────────────────

  SelfRobot robot0;          // 현재 pose 추정 (맵 픽셀 + 도)
  std::vector<Particle> pt;        // 파티클 (pose 가설)
  int particle_num = 8000;
  float particle_range = 0.0;      // 재살포 sigma (픽셀)

  MclEngine Likelihood = MclEngine();  // 관측·가중치·리샘플
  MotionModel motion;                        // 오도메트리 예측

  double weight_update_threshold = 14.0;  // 이 값 넘는 최고 가중치만 채택
  bool noise_enable = true;

  // 관측 프레임 상태 (visionFeatureCallback 이 채우고 featureCalc 이 소비)
  bool valid_observation = false;
  int vision_data_cnt = 0;
  int vision_data_size = 0;

  // 관측 시점의 공 위치 (로봇 기준 mm). 특징점-공 방향 일치 검사에 쓴다.
  // 워커가 깨어난 시점의 visionMSG 를 읽으면 이미 다음 프레임으로 덮인
  // 값일 수 있어, 관측과 같은 시점의 값을 여기 실어 보낸다.
  double ball_mm_x = 0.0;
  double ball_mm_y = 0.0;

  // ── 락 순서 규약 ───────────────────────────────────────────────
  //
  //        vision_mutex  →  pt_mutex  →  robot0_mutex
  //
  // 중첩해서 잡을 때는 반드시 이 순서를 지킬 것. 일부만 잡는 건 괜찮지만
  // 역순으로 잡으면 안 된다. 순차적으로(앞의 것을 놓고 뒤의 것을 잡기) 쓰는
  // 것도 허용된다 — ikCallback 이 그렇게 한다.
  //
  // 이 규약이 깨졌을 때 생기는 일: 예전에는 경로마다 순서가 달라
  //   featureCalc          : pt → vision → robot0
  //   visionFeatureCallback: vision → robot0
  //   visionCallback       : robot0 → pt
  // 로 pt → vision → robot0 → pt 순환이 있었다. 콜백이 전부 spin_some 단일
  // 스레드라 우연히 안 터졌을 뿐, MultiThreadedExecutor 나 콜백 그룹을
  // 도입하는 순간 데드락이 된다.

  std::mutex vision_mutex;   // Likelihood.vision_point_vect 보호
  std::mutex pt_mutex;       // pt[] 보호 (ikCallback vs 워커)
  std::mutex robot0_mutex;   // robot0 보호 (ROS 콜백 vs Qt 메인)

private:
  void loop();

  std::thread thread_;
  // 이 락은 pending_/running_ 만 보호하는 leaf 락이다. 다른 락과 겹쳐 잡지
  // 않으므로 위 vision/pt/robot0 순서 규약과 무관하다.
  std::mutex mutex_;
  std::condition_variable cv_;
  bool pending_ = false;
  bool running_ = false;
};
