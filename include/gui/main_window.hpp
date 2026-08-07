#include <QMainWindow>
#include <QGraphicsScene>
#include <QDialog>
#include <QVBoxLayout>
#include <QLabel>
#include <QtWidgets>
#include <QDir>
#include <QFile>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include "QIcon"
#include "gui/qnode.hpp"
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

    memoEdit = new QTextEdit(this);
    memoEdit->setGeometry(30, 30, 371, 271);

    // 저장된 메모 불러오기
    std::string share = ament_index_cpp::get_package_share_directory("robocup_localization");
    QString memoPath = QString::fromStdString(share) + "/resources/memo.txt";
    QFile loadFile(memoPath);
    if (loadFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      memoEdit->setPlainText(QString::fromUtf8(loadFile.readAll()));
      loadFile.close();
    }

    QPushButton *saveBtn = new QPushButton("저장", this);
    saveBtn->setGeometry(30, 315, 160, 41);

    connect(saveBtn, &QPushButton::clicked, this, [this, memoPath]() {
      QFile saveFile(memoPath);
      if (saveFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        saveFile.write(memoEdit->toPlainText().toUtf8());
        saveFile.close();
      }
    });

    QTextEdit *labelEdit = new QTextEdit(this);
    labelEdit->setGeometry(200, 315, 201, 41);
    labelEdit->setReadOnly(true);
    labelEdit->setHtml(
        "<html><head><style>p, li { white-space: pre-wrap; }</style></head>"
        "<body style=\" font-family:'Ubuntu'; font-size:11pt;\">"
        "<p align=\"center\">저장 경로: ~/.robocup_memo.txt</p>"
        "</body></html>");
  }

private:
  QTextEdit *memoEdit = nullptr;
};


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
  QPolygonF create_Print_robot(const Pose2D &robot);
  void Print_Screen_Ball();

  // For UI
  int set_robot_flag = 0;
  int setting_flag = 0;
  bool obstacle_mode = false;
  float adult_ui_size_cali = 0.0; // 어덜트 맵 크기에 따른 ui 사이즈 보정값
  float particle_range = 0.0;     // is 100

  void setting();

  // For udp

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
  void on_obstacle_trigger_clicked();
  void on_btn_info_clicked();
};


