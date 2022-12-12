#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/ui/qt/home.h"
#include "selfdrive/ui/qt/onroad.h"
#include "selfdrive/ui/qt/offroad/onboarding.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "tools/replay/replay.h"

class MainWindow : public QWidget {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);

private:
  QVBoxLayout *main_layout;

  OnroadWindow *onroad;
  std::unique_ptr<Replay> replay;

  QStringList routes = {
    "d66fbf5597dfbff4|2022-10-10--16-57-23",
    "d545129f3ca90f28|2022-10-19--09-22-54",
  };
//  bool eventFilter(QObject *obj, QEvent *event) override;
//  void openSettings(int index = 0, const QString &param = "");
//  void closeSettings();
};
