#pragma once

#include <QVBoxLayout>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QWidget>

#include "selfdrive/ui/qt/onroad.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "tools/replay/replay.h"

class MainWindowNoTouch : public QWidget {
  Q_OBJECT

public:
  explicit MainWindowNoTouch(QWidget *parent = 0);

private:
  QVBoxLayout *main_layout;

  OnroadWindow *onroad;
  std::unique_ptr<Replay> replay;

  QMediaPlayer *player;
  QVideoWidget *videoWidget;

  QMap<QString, QString> content = {
    {"Media1 - Media", "/data/tacos.mp4"},
    {"Route1 - Route", "d545129f3ca90f28|2022-12-13--22-57-16"},
  };
};
