#include "gui/odom_param_store.hpp"

#include <cstdio>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "robot_config.h"

namespace param_store
{

std::string filePath()
{
  const std::string share_dir =
      ament_index_cpp::get_package_share_directory("robocup_localization");

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
  return share_dir + "/resources/param/PARAM1.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_2)
  return share_dir + "/resources/param/PARAM2.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_3)
  return share_dir + "/resources/param/PARAM3.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_4)
  return share_dir + "/resources/param/PARAM4.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_5)
  return share_dir + "/resources/param/PARAM5.txt";
#elif defined(ROBIT_HUMANOID_ROBOT_NUMBER_6)
  return share_dir + "/resources/param/PARAM6.txt";
#else
#error "로봇 번호 미정의. robot_config.h 에 ROBIT_HUMANOID_ROBOT_NUMBER_{1..6} 중 하나를 정의하세요."
#endif
}

bool load(OdomParams &out)
{
  std::ifstream in(filePath());
  if (!in.is_open())
  {
    printf("[LocalNode] No Odom Data\n");
    return false;
  }

  // 5개 값만 읽는다. 뒤에 붙은 주석은 무시된다.
  in >> out.particle_range;
  in >> out.odom_scale_forward;
  in >> out.odom_scale_backward;
  in >> out.odom_scale_left;
  in >> out.odom_scale_right;

  printf("[LocalNode] Odom Load\n");
  return true;
}

bool save(const OdomParams &params)
{
  std::ofstream out(filePath());
  if (!out.is_open())
  {
    printf("[LocalNode] No Odom Parameter\n");
    return false;
  }

  out << params.particle_range << std::endl;
  out << params.odom_scale_forward << std::endl;
  out << params.odom_scale_backward << std::endl;
  out << params.odom_scale_left << std::endl;
  out << params.odom_scale_right << std::endl;

  printf("[LocalNode] Odom Param set\n");
  return true;
}

}  // namespace param_store
