/**
 * @file user_config_path.hpp
 * @brief 사용자가 저장한 값(오도메트리 캘리브레이션, 메모)을 어디에 둘 것인가.
 *
 * 예전에는 패키지 share 디렉터리에 바로 썼다. 그 자리는 colcon 이
 * `install(DIRECTORY resources ...)` 로 소스에서 매 빌드마다 다시 복사하는
 * 곳이라, 저장해 놓아도 다음 빌드에 조용히 덮어써졌다
 * ("저장 후 재시작하면 옛 값").
 *
 * 그래서 **소스 트리**에 쓴다. PARAM1..6 이 로봇별로 레포에 커밋돼 있는 걸
 * 보면 원래 그게 의도였다 — 캘리브레이션이 git 에 남아 로봇에 배포된다.
 * 실행 파일은 소스 경로를 알 방법이 없으므로 CMake 가 빌드 시점에
 * ROBOCUP_LOCALIZATION_SOURCE_RESOURCE_DIR 로 박아 준다.
 *
 * 소스 없이 install 만 복사해 배포한 경우에는 그 경로가 없다. 그때는 예전처럼
 * share 를 쓴다 — 그 환경에서는 재빌드가 없으니 덮어써질 일도 없다.
 */

#pragma once

#include <filesystem>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robot_config.h"  // ROBIT_HUMANOID_ROBOT_NUMBER_*

namespace robocup_localization::app
{

/// CMake 가 박아 준 소스 resources 경로. 그 자리에 실제로 디렉터리가 있을 때만
/// 돌려준다 (다른 기계에 install 만 복사한 경우 경로만 남고 실체가 없다).
inline std::string sourceResourceDir()
{
#ifdef ROBOCUP_LOCALIZATION_SOURCE_RESOURCE_DIR
  const std::string dir = ROBOCUP_LOCALIZATION_SOURCE_RESOURCE_DIR;
  std::error_code ec;
  if (std::filesystem::is_directory(dir, ec))
  {
    return dir;
  }
#endif
  return {};
}

/// 패키지가 배포한 사본. 소스가 없을 때만 쓴다.
inline std::string packageResourceDir()
{
  return ament_index_cpp::get_package_share_directory("robocup_localization") +
         "/resources";
}

/// resources 기준 상대 경로를 실제 경로로. 소스가 있으면 소스, 없으면 share.
inline std::string resourcePath(const std::string& relative)
{
  const std::string base = sourceResourceDir();
  if (!base.empty())
  {
    return (std::filesystem::path(base) / relative).string();
  }
  return packageResourceDir() + "/" + relative;
}

/// 이 로봇의 오도메트리 캘리브레이션 파일 (resources 기준 상대 경로).
/// 로봇 번호 분기는 여기 한 곳에만 둔다 — 예전에는 읽기/쓰기 두 곳에
/// 같은 #ifdef 블록이 복붙돼 있었다.
inline std::string odomParamRelativePath()
{
#if defined(ROBIT_HUMANOID_ROBOT_NUMBER_1)
  return "param/PARAM1.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_2)
  return "param/PARAM2.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_3)
  return "param/PARAM3.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_4)
  return "param/PARAM4.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_5)
  return "param/PARAM5.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_6)
  return "param/PARAM6.txt";
#else
  return {};  // 로봇 번호 미정의 — 호출부가 빈 값을 보고 알린다
#endif
}

}  // namespace robocup_localization::app
