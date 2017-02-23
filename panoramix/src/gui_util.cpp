#include "pch.hpp"

#include "cameras.hpp"

#include "color.hpp"
#include "gui_util.hpp"
#include "qttools.hpp"
#include "scene.hpp"
#include "shader.hpp"
#include "ui.hpp"

namespace pano {
namespace gui {

using namespace core;

int SelectFrom(const std::vector<std::string> &strs, const std::string &title,
               const std::string &text, int acceptId, int rejectId) {
  UI::InitGui();
  QMessageBox mbox;
  std::vector<QAbstractButton *> buttons(strs.size(), nullptr);
  for (int i = 0; i < strs.size(); i++) {
    buttons[i] = new QPushButton(QString::fromStdString(strs[i]));
    auto role = i == acceptId
                    ? QMessageBox::ButtonRole::AcceptRole
                    : (i == rejectId ? QMessageBox::ButtonRole::RejectRole
                                     : QMessageBox::ButtonRole::NoRole);
    mbox.addButton(buttons[i], role);
  }
  mbox.setWindowTitle(title.empty() ? QObject::tr("Make your decision")
                                    : QString::fromStdString(title));
  mbox.setText(text.empty() ? QObject::tr("Click on one of the buttons")
                            : QString::fromStdString(text));
  mbox.exec();
  for (int i = 0; i < buttons.size(); i++) {
    if (mbox.clickedButton() == buttons[i])
      return i;
  }
  return -1;
}

Image FileDialog::PickAnImage(const std::string &dir, std::string *picked) {
  UI::InitGui();
  auto filename = QFileDialog::getOpenFileName(
      nullptr, QObject::tr("Select an image file"), QString::fromStdString(dir),
      QObject::tr("Image Files (*.png;*.jpg;*.jpeg);;All Files (*.*)"));
  if (filename.isEmpty())
    return Image();
  if (picked) {
    *picked = filename.toStdString();
  }
  return cv::imread(filename.toStdString());
}

std::vector<Image> FileDialog::PickImages(const std::string &dir,
                                          std::vector<std::string> *picked) {
  UI::InitGui();
  auto filenames = QFileDialog::getOpenFileNames(
      nullptr, QObject::tr("Select an image file"), QString::fromStdString(dir),
      QObject::tr("Image Files (*.png;*.jpg;*.jpeg);;All Files (*.*)"));
  std::vector<Image> ims;
  for (auto &filename : filenames) {
    ims.push_back(cv::imread(filename.toStdString()));
  }
  if (picked) {
    for (auto &filename : filenames) {
      picked->push_back(filename.toStdString());
    }
  }
  return ims;
}

std::vector<Image>
FileDialog::PickAllImagesFromAFolder(const std::string &dir,
                                     std::vector<std::string> *picked) {
  UI::InitGui();
  auto folder = QFileDialog::getExistingDirectory(
      nullptr, QObject::tr("Select a folder containing images"),
      QString::fromStdString(dir));
  if (folder.isEmpty()) {
    return std::vector<Image>();
  }
  std::vector<Image> ims;
  QDirIterator it(folder, QStringList() << "*.jpg"
                                        << "*.png"
                                        << "*.jpeg",
                  QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    std::string filename = it.next().toStdString();
    ims.push_back(cv::imread(filename));
    std::cout << filename << " is read" << std::endl;
    if (picked) {
      picked->push_back(filename);
    }
  }
  return ims;
}

void FileDialog::ForEachImageFromAFolder(
    const std::string &dir,
    const std::function<bool(const std::string &impath)> &fun) {
  UI::InitGui();
  auto folder = QFileDialog::getExistingDirectory(
      nullptr, QObject::tr("Select a folder containing images"),
      QString::fromStdString(dir));
  if (folder.isEmpty()) {
    return;
  }
  QDirIterator it(folder, QStringList() << "*.jpg"
                                        << "*.png"
                                        << "*.jpeg",
                  QDir::Files, QDirIterator::Subdirectories);
  while (it.hasNext()) {
    std::string filename = it.next().toStdString();
    std::cout << "processing " << filename << std::endl;
    if (!fun(filename)) {
      break;
    }
  }
}

std::vector<std::string> FileDialog::PickFiles(const std::string &dir,
                                               const std::string &suffix) {
  UI::InitGui();
  auto filenames = QFileDialog::getOpenFileNames(
      nullptr, QObject::tr("Select files"), QString::fromStdString(dir),
      QObject::tr("Files (%1)").arg(QString::fromStdString(suffix)));
  std::vector<std::string> picked;
  for (auto &filename : filenames) {
    picked.push_back(filename.toStdString());
  }
  return picked;
}

bool MakePanoramaByHand(Image &im, bool *extendedOnTop, bool *extendedOnBottom,
                        bool *topIsPlanar, bool *bottomIsPlanar) {
  if (im.cols < im.rows * 2)
    return false;
  if (im.cols == im.rows * 2) {
    if (extendedOnTop)
      *extendedOnTop = false;
    if (extendedOnBottom)
      *extendedOnBottom = false;
    LOG("No need to adjust the panoramic image by hand");
    return true;
  }

  class Widget : public QGLWidget {
    using BaseClass = QGLWidget;

  public:
    explicit Widget(Scene &&scene, RenderOptions &options,
                    QWidget *parent = nullptr)
        : BaseClass(parent), _scene(std::move(scene)), _options(options) {

      setMouseTracking(true);
      setAutoBufferSwap(false);
      grabKeyboard();
    }

  protected:
    void initializeGL() {
      makeCurrent();
      glEnable(GL_MULTISAMPLE);
      GLint bufs;
      GLint samples;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
      glGetIntegerv(GL_SAMPLES, &samples);
      qDebug("Have %d buffers and %d samples", bufs, samples);
      qglClearColor(MakeQColor(_options.backgroundColor()));
      _scene.initialize();
    }

    void resizeGL(int w, int h) {
      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(w, h));
      glViewport(0, 0, w, h);
    }

    void paintEvent(QPaintEvent *e) override {
      QPainter painter(this);
      painter.beginNativePainting();
      qglClearColor(MakeQColor(_options.backgroundColor()));
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(width(), height()));

      _scene.render(_options);

      painter.endNativePainting();
      swapBuffers();
    }

    void mousePressEvent(QMouseEvent *e) {
      if (e->buttons() & Qt::MidButton) {
        _lastPos = e->pos();
        setCursor(Qt::OpenHandCursor);
      } else {
        BaseClass::mousePressEvent(e);
      }
    }

    void mouseMoveEvent(QMouseEvent *e) {
      QVector3D t(e->pos() - _lastPos);
      t.setX(-t.x());
      if (e->buttons() & Qt::MidButton) {
        _options.camera().moveCenterWithEyeFixed(MakeCoreVec(t));
        setCursor(Qt::ClosedHandCursor);
        update();
      } else {
        BaseClass::mouseMoveEvent(e);
      }
      _lastPos = e->pos();
    }

    void wheelEvent(QWheelEvent *e) {
      _options.camera().setFocal(_options.camera().focal() *
                                 exp(e->delta() / 1000.0));
      BaseClass::wheelEvent(e);
      update();
    }

    void keyPressEvent(QKeyEvent *e) override { BaseClass::keyPressEvent(e); }

  private:
    QPoint _lastPos;
    Scene _scene;
    RenderOptions &_options;
  };

  SceneBuilder sb;
  ResourceStore::set("tex", im);
  Sphere3 sp;
  sp.center = Origin();
  sp.radius = 1.0;
  sb.begin(sp)
      .shaderSource(OpenGLShaderSourceDescriptor::XPanorama)
      .resource("tex")
      .end();

  RenderOptions options;
  options.panoramaAspectRatio(im.rows / float(im.cols));
  options.panoramaHoriCenterRatio(0.5f);
  options.camera(
      PerspectiveCamera(500, 500, Point2(250, 250), 200, Origin(), X(), Z()));

  auto app = UI::InitGui();
  Widget *w = new Widget(sb.scene(), options);

  QMainWindow *mwin = new QMainWindow;
  mwin->setCentralWidget(w);
  mwin->setAttribute(Qt::WA_DeleteOnClose);
  mwin->resize(MakeQSize(options.camera().screenSize()));
  mwin->setWindowTitle(QString::fromStdString(options.winName()));
  mwin->setWindowIcon(UI::DefaultIcon());
  mwin->setStyleSheet(UI::DefaultCSS());

  auto menuView = mwin->menuBar()->addMenu(QObject::tr("View"));
  auto actionSettings = menuView->addAction(QObject::tr("Settings"));
  QObject::connect(actionSettings, &QAction::triggered, [w, &options]() {
    PopUpGui(options, w);
    w->update();
  });
  auto menuAbout = mwin->menuBar()->addMenu(QObject::tr("About"));
  auto actionAbout = menuAbout->addAction(QObject::tr("About"));
  QObject::connect(actionAbout, &QAction::triggered, [mwin]() {
    QMessageBox::about(
        mwin, QObject::tr("About this program"),
        QObject::tr("Panoramix.Vis is the visulization module of project "
                    "Panoramix developped by Yang Hao."));
  });
  mwin->statusBar()->show();

  auto palette = mwin->palette();
  palette.setColor(QPalette::Window, MakeQColor(options.backgroundColor()));
  mwin->setPalette(palette);

  mwin->show();
  UI::ContinueGui();

  float panoHCenterRatio = 1.0f - options.panoramaHoriCenterRatio();

  int panoHCenter = panoHCenterRatio * im.rows;
  std::cout << "hcenter ratio: " << panoHCenterRatio << std::endl;
  std::cout << "hcenter: " << panoHCenter << std::endl;

  auto result = MakePanorama(im, panoHCenter, extendedOnTop, extendedOnBottom);
  if (!result) {
    std::cerr << "input panorama shape is incorrect!" << std::endl;
    return false;
  }

  if (extendedOnTop && topIsPlanar) {
    int selected =
        pano::gui::SelectFrom({"Is Planar", "Is Not Planar"}, "Your decision?",
                              "Is TOP Region Planar?", 0, 1);
    *topIsPlanar = selected == 0;
  }
  if (extendedOnBottom && bottomIsPlanar) {
    int selected =
        pano::gui::SelectFrom({"Is Planar", "Is Not Planar"}, "Your decision?",
                              "Is BOTTOM Region Planar?", 0, 1);
    *bottomIsPlanar = selected == 0;
  }

  return true;
}

Qt::PenStyle MakeQPenStyle(PenStyle ps) { return Qt::PenStyle(ps); }

template <class BaseWidgetT> class PaintableWidget : public BaseWidgetT {
public:
  explicit PaintableWidget(const std::vector<PenConfig> &pc,
                           QWidget *parent = nullptr)
      : BaseWidgetT(parent), _activePenId(-1),
        _penCursor(QPixmap(tr(":/icons/pencil_icon&16.png")), 0, 16),
        _pens(pc.size()) {

    setMouseTracking(true);

    // setup pen selection actions
    setContextMenuPolicy(Qt::ActionsContextMenu);
    QActionGroup *bas = new QActionGroup(this);

    QAction *defaultAction = nullptr;
    connect(defaultAction = bas->addAction(tr("Pick")), &QAction::triggered,
            [this]() { _activePenId = -1; });
    {
      QAction *sep = new QAction(bas);
      sep->setSeparator(true);
      bas->addAction(sep);
    }
    for (int i = 0; i < pc.size(); i++) {
      auto action = bas->addAction(QString::fromStdString(pc[i].name));
      action->setToolTip(QString::fromStdString(pc[i].description));
      action->setWhatsThis(QString::fromStdString(pc[i].description));
      // draw icon
      int sz = 16;
      QImage image(sz, sz, QImage::Format::Format_ARGB32_Premultiplied);
      image.fill(MakeQColor(gui::White));
      _pens[i] = QPen(MakeQColor(pc[i].color), pc[i].thickness,
                      MakeQPenStyle(pc[i].style));
      QPainter painter(&image);
      painter.setPen(_pens[i]);
      painter.drawLine(QPointF(0, sz / 2), QPointF(sz, sz / 2));
      painter.end();
      action->setIcon(QIcon(QPixmap::fromImage(image)));
      connect(action, &QAction::triggered, [this, i] { _activePenId = i; });
    }

    for (auto a : bas->actions())
      a->setCheckable(true);

    bas->setExclusive(true);
    defaultAction->setChecked(true);

    addActions(bas->actions());
  }

protected:
  void mousePressEvent(QMouseEvent *e) override {
    _moved = false;
    if (e->buttons() & Qt::LeftButton) {
      if (_activePenId == -1) { // pick
        processPick(Point2(e->pos().x(), e->pos().y()));
        return;
      }
      _stroke << e->pos();
      setCursor(_penCursor);
    } else {
      QWidget::mousePressEvent(e);
    }
  }

  void mouseMoveEvent(QMouseEvent *e) override {
    _moved = true;
    if (e->buttons() & Qt::LeftButton) {
      if (_activePenId == -1)
        return;
      auto pos = e->pos();
      if (_stroke.isEmpty() || (_stroke.last() - pos).manhattanLength() > 3) {
        _stroke << pos;
        update();
      }
    } else {
      QWidget::mouseMoveEvent(e);
    }
  }

  void mouseReleaseEvent(QMouseEvent *e) override {
    bool isClick = !_moved;
    _moved = false;
    unsetCursor();
    if (_activePenId == -1)
      return;
    std::vector<Point2> points(_stroke.size());
    for (int i = 0; i < _stroke.size(); i++) {
      points[i][0] = _stroke[i].x();
      points[i][1] = _stroke[i].y();
    }
    if (!points.empty()) {
      processStroke(points, _activePenId);
      update();
    }
    _stroke.clear();
    update();
  }

  virtual void processStroke(const std::vector<Point2> &stroke, int penId) = 0;
  virtual void processPick(const Point2 &p) {}

protected:
  QPolygonF _stroke;
  int _activePenId;
  std::vector<QPen> _pens;
  std::vector<PenConfig> _penConfigs;
  bool _moved;
  QCursor _penCursor;
};

void PaintWith(const std::function<Image()> &updater,
               const std::vector<PenConfig> &penConfigs,
               const std::function<bool(const std::vector<Point2> &polyline,
                                        int penId)> &callback) {

  UI::InitGui();

  class Widget : public PaintableWidget<QWidget> {
    using BaseClass = PaintableWidget<QWidget>;

  public:
    explicit Widget(
        const std::function<Image()> &up, const std::vector<PenConfig> &pc,
        const std::function<bool(const std::vector<Point2> &, int)> &cb)
        : BaseClass(pc), _updater(up), _callback(cb) {
      _scale = 1.0;
      updateImage();
    }

    void updateImage() {
      auto im = _updater();
      _image = MakeQImage(im);
    }

  protected:
    void paintEvent(QPaintEvent *e) override {
      QPainter painter(this);
      painter.setBackground(Qt::white);
      painter.eraseRect(rect());

      painter.resetTransform();
      painter.translate(rect().center() + _translate);
      painter.scale(_scale, _scale);
      painter.translate(-_image.rect().center());

      painter.fillRect(_image.rect().translated(QPoint(5, 5) / _scale),
                       QColor(35, 30, 30, 100));
      painter.drawImage(_image.rect(), _image);
      painter.resetTransform();

      if (_activePenId >= 0) {
        auto pen = _pens[_activePenId];
        painter.setPen(pen);
      }
      painter.setRenderHint(QPainter::Antialiasing);
      painter.drawPolyline(_stroke);
    }

    void mousePressEvent(QMouseEvent *e) override {
      if (e->buttons() & Qt::MidButton) {
        // move
        _lastPos = e->pos();
        setCursor(Qt::OpenHandCursor);
        update();
      } else {
        BaseClass::mousePressEvent(e);
      }
    }

    void mouseMoveEvent(QMouseEvent *e) override {
      if (e->buttons() & Qt::MidButton) {
        setCursor(Qt::ClosedHandCursor);
        QPoint t(e->pos() - _lastPos);
        _translate += t;
        _lastPos = e->pos();
        update();
      } else {
        BaseClass::mouseMoveEvent(e);
      }
    }

    void wheelEvent(QWheelEvent *e) override {
      double d = e->delta() / 500.0;
      if (_scale >= 50.0 && d >= 0)
        return;
      if (_scale <= 0.02 && d <= 0)
        return;
      _scale *= std::pow(2.0, d);
      update();
    }

    void keyPressEvent(QKeyEvent *e) override {
      if (e->key() == Qt::Key::Key_F5) {
        updateImage();
        update();
      }
    }

    inline QPointF positionOnImage(const QPointF &screenPos) const {
      // (x - imcenter) * scale + rect.center + translate
      return (screenPos - _translate - rect().center()) / _scale +
             _image.rect().center();
    }

    void processStroke(const std::vector<Point2> &stroke, int penId) {
      if (_callback(stroke, penId)) {
        updateImage();
        update();
      }
    }

  private:
    QImage _image;
    double _scale;
    QPointF _translate;
    QPoint _lastPos;
    std::function<Image()> _updater;
    std::function<bool(const std::vector<Point2> &polyline, int penId)>
        _callback;
  };

  Widget w(updater, penConfigs, callback);
  w.show();

  UI::ContinueGui();
}

void VisualizeWithPanoramicOperation(const Scene &scene,
                                     const RenderOptions &options) {

  using namespace pano::core;

  UI::InitGui();

  class Widget : public PaintableWidget<QGLWidget> {
    using BaseClass = PaintableWidget<QGLWidget>;

  public:
    explicit Widget(const Scene &scene, const RenderOptions &options,
                    QWidget *parent = nullptr)
        : BaseClass({}, parent), _scene(scene), _options(options) {

      setMouseTracking(true);
      setAutoBufferSwap(false);
      grabKeyboard();
    }

  protected:
    void initializeGL() {
      makeCurrent();
      glEnable(GL_MULTISAMPLE);
      GLint bufs;
      GLint samples;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
      glGetIntegerv(GL_SAMPLES, &samples);
      qDebug("Have %d buffers and %d samples", bufs, samples);
      qglClearColor(MakeQColor(_options.backgroundColor()));
      _scene.initialize();
    }

    void resizeGL(int w, int h) {
      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(w, h));
      glViewport(0, 0, w, h);
    }

    void paintEvent(QPaintEvent *e) override {
      QPainter painter(this);
      painter.beginNativePainting();
      qglClearColor(MakeQColor(_options.backgroundColor()));
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(width(), height()));

      _scene.render(_options);

      painter.endNativePainting();

      if (_activePenId >= 0) {
        auto pen = _pens[_activePenId];
        painter.setPen(pen);
      }
      painter.setRenderHint(QPainter::Antialiasing);
      painter.drawPolyline(_stroke);
      swapBuffers();
    }

    void mousePressEvent(QMouseEvent *e) {
      if (e->buttons() & Qt::MidButton) {
        _lastPos = e->pos();
        setCursor(Qt::OpenHandCursor);
      } else {
        BaseClass::mousePressEvent(e);
      }
    }

    void mouseMoveEvent(QMouseEvent *e) {
      QVector3D t(e->pos() - _lastPos);
      t.setX(-t.x());
      if (e->buttons() & Qt::MidButton) {
        _options.camera().moveCenterWithEyeFixed(MakeCoreVec(t));
        setCursor(Qt::ClosedHandCursor);
        update();
      } else {
        BaseClass::mouseMoveEvent(e);
      }
      _lastPos = e->pos();
    }

    void wheelEvent(QWheelEvent *e) {
      _options.camera().setFocal(_options.camera().focal() *
                                 exp(e->delta() / 1000.0));
      BaseClass::wheelEvent(e);
      update();
    }

    void keyPressEvent(QKeyEvent *e) override { BaseClass::keyPressEvent(e); }

    virtual void processStroke(const std::vector<Point2> &stroke,
                               int penId) override {}

    virtual void processPick(const Point2 &p) override {
      _scene.invokeCallbackFunctions(gui::InteractionID::ClickLeftButton,
                                     _scene.pickOnScreen(_options, p), false);
    }

  private:
    QPoint _lastPos;
    const Scene &_scene;
    RenderOptions _options;
  };

  Widget w(scene, options);
  w.show();

  UI::ContinueGui();
}

void DrawChainsInPanorama(const PanoramicView &view,
                          const std::vector<PenConfig> &penConfigs,
                          std::vector<Chain3> &chains) {
  using namespace pano::core;

  UI::InitGui();

  class Widget : public UIWidget<QGLWidget> {
    using BaseClass = UIWidget<QGLWidget>;

  public:
    explicit Widget(const std::vector<PenConfig> &pc, const PanoramicView &v,
                    std::vector<Chain3> &chains, QWidget *parent = nullptr)
        : BaseClass(pc, parent), _view(v), _chains(chains),
          _selectedCornerId(-1) {

      if (_chains.empty()) {
        _chains.resize(pc.size());
      }

      setMouseTracking(true);
      setAutoBufferSwap(false);
      grabKeyboard();

      // build scene
      SceneBuilder sb;
      ResourceStore::set("tex", v.image);
      sb.begin(UnitSphere())
          .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
          .resource("tex")
          .end();
      _scene = sb.scene();

      _options.camera() = PerspectiveCamera(800, 800);
      _options.camera().setEye(Point3(0, 0, 0));
      _options.camera().setCenter(v.camera.center());
      _options.camera().setUp(-v.camera.up());
    }

  public:
    const std::vector<Chain3> &chains() const { return _chains; }

  protected:
    void initializeGL() {
      makeCurrent();
      glEnable(GL_MULTISAMPLE);
      GLint bufs;
      GLint samples;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
      glGetIntegerv(GL_SAMPLES, &samples);
      qDebug("Have %d buffers and %d samples", bufs, samples);
      qglClearColor(MakeQColor(_options.backgroundColor()));
      _scene.initialize();
    }

    void resizeGL(int w, int h) {
      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(w, h));
      glViewport(0, 0, w, h);
    }

    void paintEvent(QPaintEvent *e) override {
      QPainter painter;
      painter.begin(this);
      painter.beginNativePainting();
      qglClearColor(MakeQColor(_options.backgroundColor()));
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      auto &camera = _options.camera();
      camera.resizeScreen(Size(width(), height()));

      _scene.render(_options);

      painter.endNativePainting();

      painter.setRenderHint(QPainter::Antialiasing);

      // get visible part
      std::vector<QPolygonF> ps(_chains.size());
      for (int i = 0; i < _chains.size(); i++) {
        auto &pl = ps[i];
        auto &c = _chains[i];
        if (c.empty())
          continue;
        // visible part
        int startId = 0;
        while (!(!_options.camera().isVisibleOnScreen(c.prev(startId)) &&
                 _options.camera().isVisibleOnScreen(c.at(startId))) &&
               startId < c.size()) {
          startId++;
        }
        if (startId == c.size())
          startId = 0;
        for (int k = 0; k < c.size() &&
                        _options.camera().isVisibleOnScreen(c.at(startId + k));
             k++) {
          int id = startId + k;
          auto p = _options.camera().toScreen(c.at(id));
          pl.append(MakeQPointF(p));
        }
      }
      for (int i = 0; i < _chains.size(); i++) {
        painter.setPen(pen(i));
        painter.drawPolyline(ps[i]);
        for (auto &p : ps[i]) {
          painter.drawEllipse(p, 10, 10);
        }
      }
      swapBuffers();
    }

    void mousePressEvent(QMouseEvent *e) {
      if (e->buttons() & Qt::MidButton) {
        _lastPos = e->pos();
        setCursor(Qt::OpenHandCursor);
      } else if (e->buttons() & Qt::LeftButton) {
        auto d =
            normalize(_options.camera().toSpace(MakeCoreVec(e->pos()))) / 2;
        // click on what?

        if (activePenId() != -1) {
          auto &c = _chains[activePenId()];
          double angle = 10 / _options.camera().focal();

          // any existing corner?
          _selectedCornerId = -1;
          for (int i = 0; i < c.size(); i++) {
            double a = AngleBetweenDirected(c[i], d);
            if (a < angle) {
              _selectedCornerId = i;
              angle = a;
            }
          }

          if (_selectedCornerId == -1) {
            // any edge ?
            angle = 5 / _options.camera().focal();
            int nid = -1;
            for (int i = 0; i < c.size(); i++) {
              auto nn = DistanceFromPointToLine(d, c.edge(i)).second.position;
              double a = AngleBetweenDirected(d, nn);
              if (a < angle) {
                angle = a;
                nid = i;
              }
            }
            if (nid != -1) {
              c.insert(nid + 1, d);
              _selectedCornerId = (nid + 1) % c.size();
            }
          }

          if (_selectedCornerId == -1) {
            c.append(d);
            _selectedCornerId = c.size() - 1;
          }
        }

      } else {
        BaseClass::mousePressEvent(e);
      }
      update();
    }

    void mouseMoveEvent(QMouseEvent *e) {
      QVector3D t(e->pos() - _lastPos);
      t.setX(-t.x());
      if (e->buttons() & Qt::MidButton) {
        _options.camera().moveCenterWithEyeFixed(MakeCoreVec(t));
        setCursor(Qt::ClosedHandCursor);
        update();
      } else if (e->buttons() & Qt::LeftButton) {
        if (_selectedCornerId != -1 && activePenId() != -1) {
          auto &c = _chains[activePenId()];
          auto d =
              normalize(_options.camera().toSpace(MakeCoreVec(e->pos()))) / 2;
          c[_selectedCornerId] = d;
        }
      } else {
        BaseClass::mouseMoveEvent(e);
      }
      _lastPos = e->pos();
      update();
    }

    void wheelEvent(QWheelEvent *e) {
      _options.camera().setFocal(_options.camera().focal() *
                                 exp(e->delta() / 1000.0));
      BaseClass::wheelEvent(e);
      update();
    }

  private:
    QPoint _lastPos;
    PanoramicView _view;
    Scene _scene;
    RenderOptions _options;
    std::vector<Chain3> &_chains;
    int _selectedCornerId;
  };

  Widget w(penConfigs, view, chains);
  w.show();

  UI::ContinueGui();
}

void VisualizeAll(const View<PanoramicCamera, Image3ub> &view,
                  const std::vector<Classified<Line3>> &lines,
                  const Imagei &segs, int nsegs, const Image5d &gc) {

  using namespace core;

  UI::InitGui();

  auto segim = CreateRandomColorTableWithSize(nsegs)(segs);
  segim = (segim * 2.0 + view.image) / 3.0;

  class Widget : public QGLWidget {
    using BaseClass = QGLWidget;

  public:
    explicit Widget(const PanoramicView &v,
                    const std::vector<Classified<Line3>> &lines,
                    QWidget *parent = nullptr)
        : BaseClass(parent), _view(v), _lines(lines) {

      setMouseTracking(true);
      setAutoBufferSwap(false);
      grabKeyboard();

      // build scene
      SceneBuilder sb;
      ResourceStore::set("tex", v.image);

      sb.begin(UnitSphere())
          .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
          .resource("tex")
          .end();
      _scene = sb.scene();

      _options.camera() = PerspectiveCamera(800, 800);
      _options.camera().setEye(Point3(0, 0, 0));
      _options.camera().setCenter(v.camera.center());
      _options.camera().setUp(-v.camera.up());
    }

  protected:
    void initializeGL() {
      makeCurrent();
      glEnable(GL_MULTISAMPLE);
      GLint bufs;
      GLint samples;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
      glGetIntegerv(GL_SAMPLES, &samples);
      qDebug("Have %d buffers and %d samples", bufs, samples);
      qglClearColor(MakeQColor(_options.backgroundColor()));
      _scene.initialize();
    }

    void resizeGL(int w, int h) {
      PerspectiveCamera &camera = _options.camera();
      camera.resizeScreen(Size(w, h));
      glViewport(0, 0, w, h);
    }

    void paintEvent(QPaintEvent *e) override {
      QPainter painter;
      painter.begin(this);
      painter.beginNativePainting();
      qglClearColor(MakeQColor(_options.backgroundColor()));
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      auto &camera = _options.camera();
      camera.resizeScreen(Size(width(), height()));

      _scene.render(_options);

      painter.endNativePainting();
      painter.setRenderHint(QPainter::Antialiasing);

      // get visible part
      QPen pens[] = {QPen(Qt::red, 2.0), QPen(Qt::green, 2.0),
                     QPen(Qt::blue, 2.0), QPen(Qt::white, 2.0)};
      for (int i = 0; i < _lines.size(); i++) {
        auto &l = _lines[i];
        if (_options.camera().isVisibleOnScreen(l.component.first) &&
            _options.camera().isVisibleOnScreen(l.component.second)) {
          if (l.claz == -1) {
            painter.setPen(pens[3]);
          } else {
            painter.setPen(pens[l.claz]);
          }
          painter.drawLine(
              MakeQPointF(_options.camera().toScreen(l.component.first)),
              MakeQPointF(_options.camera().toScreen(l.component.second)));
        }
      }
      swapBuffers();
    }

    void mousePressEvent(QMouseEvent *e) {
      if (e->buttons() & Qt::MidButton) {
        _lastPos = e->pos();
        setCursor(Qt::OpenHandCursor);
      } else {
        BaseClass::mousePressEvent(e);
      }
      update();
    }

    void mouseMoveEvent(QMouseEvent *e) {
      QVector3D t(e->pos() - _lastPos);
      t.setX(-t.x());
      if (e->buttons() & Qt::MidButton) {
        _options.camera().moveCenterWithEyeFixed(MakeCoreVec(t));
        setCursor(Qt::ClosedHandCursor);
        update();
      } else {
        BaseClass::mouseMoveEvent(e);
      }
      _lastPos = e->pos();
      update();
    }

    void wheelEvent(QWheelEvent *e) {
      _options.camera().setFocal(_options.camera().focal() *
                                 exp(e->delta() / 1000.0));
      BaseClass::wheelEvent(e);
      update();
    }

  private:
    QPoint _lastPos;
    PanoramicView _view;
    Scene _scene;
    RenderOptions _options;
    const std::vector<Classified<Line3>> &_lines;
  };

  Widget w(PanoramicView(segim, view.camera), lines);
  w.show();

  UI::ContinueGui();
}
}
}
