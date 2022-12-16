#pragma once

#include <QVBoxLayout>
#include <QMediaPlayer>
#include <QMediaPlaylist>
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

  QMap<QString, QString> content = {
    {"TB - Media", "../../../tacos.mp4"},
    {"three B-roll - Media", "../assets/videos/out/three-rotating.mp4"},
    {"website home B-roll - Media", "../assets/videos/out/website-home-video.mp4"},
//    {"Route1 - Route", "d545129f3ca90f28|2022-12-13--22-57-16"},
  };
};
