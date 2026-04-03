/**
 * @file /include/robocup_localization25/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef robocup_localization_MAIN_WINDOW_H
#define robocup_localization_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QGraphicsScene>
#include <QDialog>
#include <QVBoxLayout>
#include <QLabel>
#include <QtWidgets>
#include <vector>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

class Popup : public QDialog
{
  Q_OBJECT
public:
  explicit Popup(QWidget *parent = nullptr) : QDialog(parent)
  {
    setWindowTitle("Dialog");
    setFixedSize(431, 362);

    QPalette pal;
    pal.setColor(QPalette::Window, QColor(42, 39, 42));
    pal.setColor(QPalette::WindowText, Qt::white);
    pal.setColor(QPalette::Base, QColor(0, 0, 0));
    pal.setColor(QPalette::Text, Qt::white);
    pal.setColor(QPalette::Button, QColor(42, 39, 42));
    pal.setColor(QPalette::ButtonText, Qt::white);
    setPalette(pal);

    QTextEdit *textEdit_2 = new QTextEdit(this);
    textEdit_2->setGeometry(30, 30, 371, 271);

    QTextEdit *textEdit = new QTextEdit(this);
    textEdit->setGeometry(200, 320, 231, 41);
    textEdit->setHtml(
      "<html><head><style>p, li { white-space: pre-wrap; }</style></head>"
      "<body style=\" font-family:'Ubuntu'; font-size:11pt;\">"
      "<p align=\"center\">please put the manual of this App in this page</p>"
      "</body></html>"
    );
  }
};

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char **argv, QWidget *parent = nullptr);
  ~MainWindow();
  QNode *qnode;



  // For timer
  QTimer *m_Timer;
  QTimer *m_DataTimer;
  QTimer *m_screenTimer;

  // For Screen
  QPixmap buf;
  QGraphicsScene *scene;

  int cvt_Print_xy(float target);
  QPolygonF create_Print_robot(ROBOT robot);
  void Print_Screen_Ball();


  // For UI
  int set_robot_flag = 0;
  int setting_flag = 0;
  float adult_ui_size_cali = 0.0; // 어덜트 맵 크기에 따른 ui 사이즈 보정값
  float particle_range = 0.0; // is 100

  void setting();

  // For udp
  void select_ball();

private:
  Ui::MainWindowDesign *ui;
  void closeEvent(QCloseEvent *event);

public slots:
  void Print_Screen();
  void main();
  void mouseReleaseEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void on_btn_free_set_clicked();
  void on_btn_objects_save_clicked();
  void on_btn_test_clicked();
  void on_btn_set_1_clicked();
  void on_btn_set_2_clicked();
  void on_btn_set_3_clicked();
  void on_btn_set_4_clicked();
  void on_btn_set_5_clicked();
  void on_btn_set_6_clicked();
  void on_btn_set_auto_clicked();
  void on_btn_ball_set_clicked();
  void on_btn_ball_del_clicked();
  void on_btn_info_clicked();
  void featureCalc();
};

#endif // robocup_localization25_MAIN_WINDOW_H
