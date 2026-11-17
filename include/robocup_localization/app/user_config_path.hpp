// User-saved files go to the source tree: share/ is overwritten by colcon on every build.
// Falls back to share/ when only install/ was deployed (no rebuilds there).
#pragma once

#include <filesystem>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robot_config.h"

namespace robocup_localization::app
{

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

inline std::string packageResourceDir()
{
  return ament_index_cpp::get_package_share_directory("robocup_localization") +
         "/resources";
}

inline std::string resourcePath(const std::string& relative)
{
  const std::string base = sourceResourceDir();
  if (!base.empty())
  {
    return (std::filesystem::path(base) / relative).string();
  }
  return packageResourceDir() + "/" + relative;
}

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
  return {};
#endif
}

}
