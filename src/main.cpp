#include <QApplication>

#include "robocup_localization/ros/qnode.hpp"
#include "robocup_localization/ui/main_window.hpp"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QNode qnode(argc, argv);
  MainWindow window(qnode, qnode);
  QObject::connect(
      &qnode, SIGNAL(rosShutDown()), &window, SLOT(close()));

  window.show();
  return app.exec();
}
