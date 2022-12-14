#include <sys/resource.h>

#include <QApplication>
#include <QtWidgets>
#include <QTimer>
#include <QGraphicsScene>
#include <QWidget>
#include <QVBoxLayout>

#include "system/hardware/hw.h"
#include "cereal/messaging/messaging.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/notouch.h"
#include "selfdrive/ui/qt/qt_window.h"

//class MainWindowNoTouch : public QWidget {
//  Q_OBJECT
//
//public:
//  explicit MainWindowNoTouch(QWidget *parent = 0) : QWidget(parent) {
////    main_layout = new QVBoxLayout(this);
////    main_layout->setMargin(0);
//////    QLabel* hello = new QLabel("Hello!");
//////    main_layout->addWidget(hello);
//  };
//
////private:
////  bool eventFilter(QObject *obj, QEvent *event) override;
////  void openSettings(int index = 0, const QString &param = "");
////  void closeSettings();
////
////  Device device;
////
////  QVBoxLayout *main_layout;
////  HomeWindow *homeWindow;
////  SettingsWindow *settingsWindow;
////  OnboardingWindow *onboardingWindow;
//};

MainWindowNoTouch::MainWindowNoTouch(QWidget *parent) : QWidget(parent) {
  setpriority(PRIO_PROCESS, 0, -20);

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(0);
//  QLabel* hello = new QLabel("Hello!");
//  main_layout->addWidget(hello);

  QString content_name;
  while ((content_name = MultiOptionDialog::getSelection(tr("Select content"), content.keys(), "", this)).isEmpty()) {
    qDebug() << "No content selected!";
  }

  bool is_media = !content_name.contains("Route");

  content_name = content[content_name];  // get actual route/file name
  qDebug() << "Selected:" << content_name;

  if (is_media) {
    // do


    player = new QMediaPlayer;
    videoWidget = new QVideoWidget;
    player->setVideoOutput(videoWidget);

    player->setMedia(QUrl::fromLocalFile("/home/batman/Downloads/tacos.mp4"));
    player->setVolume(0);
    player->play();
    main_layout->addWidget(videoWidget);
//    videoWidget->show();


  } else {
    onroad = new OnroadWindow(this);
    main_layout->addWidget(onroad);

    QString data_dir = QString::fromStdString(Path::log_root());
    replay.reset(new Replay(content_name, {}, {}, uiState()->sm.get(), REPLAY_FLAG_ECAM, data_dir));

    if (replay->load()) {
      qDebug() << "Starting replay!";
      replay->start();
    }
  }

  // no outline to prevent the focus rectangle
  setStyleSheet(R"(
    * {
      font-family: Inter;
      outline: none;
    }
  )");
  setAttribute(Qt::WA_NoSystemBackground);
}

int main(int argc, char *argv[]) {
  initApp(argc, argv);
  QApplication a(argc, argv);
  MainWindowNoTouch w;
  setMainWindow(&w);

  return a.exec();
}
