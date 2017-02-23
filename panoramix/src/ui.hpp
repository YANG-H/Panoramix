#pragma once

class QIcon;
class QString;
class QApplication;

namespace pano {
namespace gui {
struct UI {
  static const QIcon &DefaultIcon();
  static const QString &DefaultCSS();

  static void SetCmdArgs(int argc, char **argv, char **envp);
  static QApplication *InitGui(int argc, char **argv);
  static QApplication *InitGui();

  static int ContinueGui();
};
}
}
