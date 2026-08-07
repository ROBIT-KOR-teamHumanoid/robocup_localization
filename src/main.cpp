/**
 * @file /src/main.cpp
 *
 * @brief 진입점. ROS 를 먼저 올리고, GUI 는 선택적으로 붙인다.
 *
 * 예전에는 QApplication → MainWindow → QNode 순으로 소유가 이어져서
 * 디스플레이가 없으면 위치추정 자체가 뜨지 않았다. 지금은 rclcpp::init 이
 * 여기서 먼저 돌고, 화면이 필요 없으면 QNode 만 띄운다.
 **/

#include <cstdlib>
#include <cstring>

#include <QApplication>
#include <QCoreApplication>

#include <rclcpp/rclcpp.hpp>

#include "gui/main_window.hpp"
#include "gui/qnode.hpp"

namespace
{
// --headless 인자가 있거나 표시할 디스플레이가 없으면 GUI 를 띄우지 않는다.
bool wantsHeadless(int argc, char **argv)
{
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--headless") == 0)
      return true;
  }

  const char *x11 = std::getenv("DISPLAY");
  const char *wayland = std::getenv("WAYLAND_DISPLAY");
  const bool has_display = (x11 && *x11) || (wayland && *wayland);
  return !has_display;
}
}  // namespace

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (wantsHeadless(argc, argv))
  {
    // 위젯 없이 이벤트 루프만 — QNode 는 QThread 라 QCoreApplication 이면 된다.
    QCoreApplication app(argc, argv);
    QNode node;
    QObject::connect(&node, &QNode::rosShutDown, &app, &QCoreApplication::quit);
    return app.exec();
  }

  QApplication app(argc, argv);
  MainWindow w(argc, argv);
  w.show();
  return app.exec();
}
