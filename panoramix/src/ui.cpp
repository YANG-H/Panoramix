#include "pch.hpp"

#include "qttools.hpp"
#include "ui.hpp"

namespace pano {
namespace gui {

static QIcon defaultIcon;
const QIcon &UI::DefaultIcon() { return defaultIcon; }
static QString defaultCSS;
const QString &UI::DefaultCSS() { return defaultCSS; }

static int _argc = 1;
static char **_argv;
static char **_envp;

void UI::SetCmdArgs(int argc, char **argv, char **envp) {
  _argc = argc;
  _argv = argv;
  _envp = envp;
}

QApplication *UI::InitGui(int argc, char **argv) {
  if (qApp)
    return qApp;

  auto p = qgetenv("QT_QPA_PLATFORM_PLUGIN_PATH");
  std::cout << "QT_QPA_PLATFORM_PLUGIN_PATH=" << p.toStdString() << std::endl;

  // Q_IMPORT_PLUGIN(QWindowsIntegrationPlugin);
  Q_INIT_RESOURCE(gui);
  // Q_IMPORT_PLUGIN(QWindowsIntegrationPlugin)
  _argc = argc;
  _argv = argv;

  QApplication *app = new QApplication(argc, argv);

  defaultIcon = QIcon(":/icons/icon.png");
  Q_ASSERT(!defaultIcon.isNull());
  QFile file(":/css/gui_win.css");
  bool opened = file.open(QFile::ReadOnly);
  Q_ASSERT(opened);
  defaultCSS = QTextStream(&file).readAll();
  QApplication::setWindowIcon(defaultIcon);
  app->setStyleSheet(defaultCSS);

  app->setQuitOnLastWindowClosed(true);

  // set opengl version
  QGLFormat glf = QGLFormat::defaultFormat();
  glf.setVersion(2, 0); // whatever version
  glf.setProfile(QGLFormat::CoreProfile);
  qDebug("OpenGL version: %d.%d", glf.majorVersion(), glf.minorVersion());
  glf.setSampleBuffers(true);
  glf.setSamples(16);
  QGLFormat::setDefaultFormat(glf);
  return app;
}

QApplication *UI::InitGui() { return InitGui(_argc, _argv); }

int UI::ContinueGui() {
  if (!qApp) {
    qDebug() << "call InitGui first!";
    return 0;
  }
  qApp->setQuitOnLastWindowClosed(true);
  return qApp->exec();
}
}
}