#include "pch.hpp"

#include "qttools.hpp"
#include "pi_graph_annotation_widgets.hpp"

namespace pano {
namespace experimental {

using namespace pano::gui;

static const double visualDepthImage = 100.0;
static const double visualDepthFace = 9.0;
static const double visualDepthFaceCenter = 5.0;
static const double visualDepthFaceCenterLines = 5.0;
static const double visualDepthFaceCoplanarLine = 4.0;
static const double visualDepthBorder = 1.0;
static const double visualDepthClutter = 0.7;
static const double visualDepthVP = 0.6;

static const double visualBorderWidth = 5;
static const double visualCornerRadius = 10.0;

PILayoutAnnotationWidget::PILayoutAnnotationWidget(
    QWidget *parent /*= nullptr*/)
    : QGLWidget(parent), _anno(nullptr), _state(Pick) {

  _showLayouts = _showVPs = true;
  _cornerClicked = _borderClicked = _faceClicked = -1;

  setMouseTracking(true);
  setAutoBufferSwap(false);
  grabKeyboard();

  _stroke.closed = false;
  setContextMenuPolicy(Qt::ActionsContextMenu);

  QAction *defaultAction = nullptr;

  // set mode
  {
    QActionGroup *bas = new QActionGroup(this);
    connect(_actPick = bas->addAction(tr("Pick Mode")), &QAction::triggered,
            [this]() {
              _state = Pick;
              clearStroke();
              update();
            });
    _actPick->setShortcut(tr("q"));
    connect(_actDrawBorder = bas->addAction(tr("Draw Border")),
            &QAction::triggered, [this]() {
              _state = DrawingBorder;
              clearStroke();
              update();
            });
    _actDrawBorder->setShortcut(tr("w"));
    /*  connect(_actDrawClutter = bas->addAction(tr("Draw Clutter")),
      &QAction::triggered, [this]() {
          _state = DrawingClutter;
          clearStroke();
          update();
      });
      _actDrawClutter->setShortcut(tr("e"));*/
    connect(_actConnectCoplanarFaces =
                bas->addAction(tr("Connect Coplanar Faces")),
            &QAction::triggered, [this]() {
              _state = ConnectCoplanarFaces;
              clearStroke();
              update();
            });
    _actConnectCoplanarFaces->setShortcut(tr("a"));
    for (auto a : bas->actions())
      a->setCheckable(true);
    bas->setExclusive(true);
    addActions(bas->actions());
  }

  {
    QAction *sep = new QAction(this);
    sep->setSeparator(true);
    addAction(sep);
  }

  // set face orientations
  connect(defaultAction = new QAction(tr("Make Face Toward Red VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {0, -1, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("r"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Toward Green VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {1, -1, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("g"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Toward Blue VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {2, -1, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("b"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Along Red VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {-1, 0, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("Ctrl+r"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Along Green VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {-1, 1, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("Ctrl+g"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Along Blue VP"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {-1, 2, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("Ctrl+b"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Face Free"), this),
          &QAction::triggered, [this]() {
            if (_faceClicked != -1) {
              _anno->face2control[_faceClicked] = {-1, -1, true};
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("Ctrl+f"));
  addAction(defaultAction);

  {
    QAction *sep = new QAction(this);
    sep->setSeparator(true);
    addAction(sep);
  }

  // set border occlusion
  connect(defaultAction = new QAction(tr("Make Border Disconnected"), this),
          &QAction::triggered, [this]() {
            if (_borderClicked != -1) {
              _anno->border2connected[_borderClicked] = false;
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("d"));
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Make Border Connected"), this),
          &QAction::triggered, [this]() {
            if (_borderClicked != -1) {
              _anno->border2connected[_borderClicked] = true;
              rebuildLayoutScene();
            }
            update();
          });
  defaultAction->setShortcut(tr("c"));
  addAction(defaultAction);

  {
    QAction *sep = new QAction(this);
    sep->setSeparator(true);
    addAction(sep);
  }

  // visibility
  connect(defaultAction = new QAction(tr("Show Layouts"), this),
          &QAction::triggered, [this](bool checked) {
            _showLayouts = checked;
            update();
          });
  defaultAction->setShortcut(tr("1"));
  defaultAction->setCheckable(true);
  defaultAction->setChecked(_showLayouts);
  addAction(defaultAction);
  /* connect(defaultAction = new QAction(tr("Show Clutters"), this),
   &QAction::triggered, [this](bool checked) {
       _showClutters = checked;
       update();
   });
   defaultAction->setShortcut(tr("2"));
   defaultAction->setCheckable(true);
   defaultAction->setChecked(_showClutters);
   addAction(defaultAction);*/
  connect(defaultAction = new QAction(tr("Show VPs"), this),
          &QAction::triggered, [this](bool checked) {
            _showVPs = checked;
            update();
          });
  defaultAction->setShortcut(tr("3"));
  defaultAction->setCheckable(true);
  defaultAction->setChecked(_showVPs);
  addAction(defaultAction);

  {
    QAction *sep = new QAction(this);
    sep->setSeparator(true);
    addAction(sep);
  }

  // settings
  connect(defaultAction = new QAction(tr("Settings"), this),
          &QAction::triggered, [this]() {
            PopUpGui(_options, this);
            update();
          });
  addAction(defaultAction);
  connect(defaultAction = new QAction(tr("Rebuild Faces"), this),
          &QAction::triggered, [this]() {
            _anno->regenerateFaces();
            rebuildLayoutScene();
            clearStroke();
            update();
          });
  defaultAction->setShortcut(tr("Ctrl+Shift+F"));
  addAction(defaultAction);

  {
    QAction *sep = new QAction(this);
    sep->setSeparator(true);
    addAction(sep);
  }

  // save
  connect(defaultAction = new QAction(tr("Save Current Annotation"), this),
          &QAction::triggered, [this]() {
            if (_imagePath) {
              SaveLayoutAnnotation(_imagePath->toStdString(), *_anno);
            }
          });
  defaultAction->setShortcut(tr("Ctrl+s"));
  addAction(defaultAction);
}

PILayoutAnnotationWidget::~PILayoutAnnotationWidget() {}

void PILayoutAnnotationWidget::setCurAnnotation(PILayoutAnnotation *anno,
                                                const QString *imagePath) {
  _state = Pick;
  _anno = anno;
  _imagePath = imagePath;
  _lastHitCornerId = -1;
  _lastHitFaceId = -1;

  _cornerClicked = _borderClicked = _faceClicked = -1;

  assert(!anno->view.image.empty());
  auto im = anno->view.image;

  // build image scene
  SceneBuilder sb;
  ResourceStore::set("tex", anno->rectifiedImage);
  Sphere3 sp;
  sp.center = Origin();
  sp.radius = visualDepthImage;
  sb.begin(sp)
      .shaderSource(OpenGLShaderSourceDescriptor::XPanorama)
      .resource("tex")
      .end();
  _imageScene = sb.scene();

  // build vps scene
  sb.clear();
  gui::ColorTable rgb = gui::RGBGreys;
  for (int i = 0; i < anno->vps.size(); i++) {
    sb.installingOptions().discretizeOptions.color(rgb[i].blendWith(gui::White, 0.4));
    sb.begin(normalize(anno->vps[i]) * visualDepthVP)
        .shaderSource(OpenGLShaderSourceDescriptor::XPoints)
        .pointSize(40)
        .end();
    sb.begin(-normalize(anno->vps[i]) * visualDepthVP)
        .shaderSource(OpenGLShaderSourceDescriptor::XPoints)
        .pointSize(40)
        .end();
  }
  _vpsScene = sb.scene();

  _options.panoramaAspectRatio(im.rows / float(im.cols));
  _options.panoramaHoriCenterRatio(0.5f);
  _options.camera(
      PerspectiveCamera(500, 500, Point2(250, 250), 200, Origin(), X(), Z()));
  _options.renderMode(gui::RenderModeFlag::All);
  _options.cullBackFace(false);
  _options.cullFrontFace(false);
  _options.bwColor(0.5);
  _options.bwTexColor(0.5);

  rebuildLayoutScene();
  rebuildCluttersScene();

  update();
}

void PILayoutAnnotationWidget::paintEvent(QPaintEvent *e) {
  if (!_anno) {
    return;
  }
  _imageScene.initialize();
  _layoutScene.initialize();
  _cluttersScene.initialize();
  _vpsScene.initialize();
  _strokeScene.initialize();

  QPainter painter(this);
  painter.beginNativePainting();

  glEnable(GL_ALPHA_TEST);
  glEnable(GL_MULTISAMPLE);
  GLint bufs;
  GLint samples;
  glGetIntegerv(GL_SAMPLE_BUFFERS, &bufs);
  glGetIntegerv(GL_SAMPLES, &samples);

  qglClearColor(MakeQColor(_options.backgroundColor()));
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  core::PerspectiveCamera &camera = _options.camera();
  camera.resizeScreen(core::Size(width(), height()));

  _imageScene.render(_options);
  if (_showLayouts) {
    _layoutScene.render(_options);
  }
  if (_showClutters) {
    _cluttersScene.render(_options);
  }
  if (_showVPs) {
    _vpsScene.render(_options);
  }
  _strokeScene.render(_options);

  painter.endNativePainting();
  swapBuffers();
}

void PILayoutAnnotationWidget::mousePressEvent(QMouseEvent *e) {
  if (!_anno) {
    return;
  }
  if (e->buttons() & Qt::MidButton) {
    _lastPos = e->pos();
    setCursor(Qt::OpenHandCursor);
  } else if ((e->buttons() & Qt::LeftButton) && _showLayouts) {
    Vec3 dir = normalize(_options.camera().toSpace(MakeCoreVec(e->pos())));
    Ray3 ray(_options.camera().eye(), dir);

    _cornerClicked = _borderClicked = _faceClicked = -1;

    // select layout components and invoke callback functions
    auto ents = _layoutScene.pickOnScreen(
        _options, core::Point2(e->pos().x(), e->pos().y()));
    _layoutScene.invokeCallbackFunctions(InteractionID::ClickLeftButton, ents,
                                         false);

    if (_state == Pick) {

      // select something
      if (e->modifiers() & Qt::ControlModifier) {
        for (auto &ent : ents) {
          _layoutScene.switchSelect(ent);
        }
      } else {
        _layoutScene.clearSelection();
        for (auto &ent : ents) {
          _layoutScene.select(ent);
        }
      }

    } else if (_state == DrawingBorder) {
      auto &corners = _anno->corners;
      if (_cornerClicked != -1) { // yes, click on a corner
        if (_lastHitCornerId != -1) {
          // connect a border!
          _anno->addBorder(_lastHitCornerId, _cornerClicked);
          _lastHitCornerId = _cornerClicked;
        } else {
          _lastHitCornerId = _cornerClicked;
        }
      } else if (_borderClicked != -1) { // yes, click on a border
        // make a new corner here
        // break this border into two
        _anno->corners.push_back(dir);
        int newCorner = _anno->corners.size() - 1;
        _anno->splitBorderBy(_borderClicked, newCorner);
        if (_lastHitCornerId != -1) {
          // connect a border!
          _anno->addBorder(_lastHitCornerId, newCorner);
          _lastHitCornerId = newCorner;
        } else {
          _lastHitCornerId = newCorner;
        }
      } else { // a new corner not on any border
        _anno->corners.push_back(dir);
        int newCorner = _anno->corners.size() - 1;
        if (_lastHitCornerId != -1) {
          // connect a border!
          _anno->addBorder(_lastHitCornerId, newCorner);
          _lastHitCornerId = newCorner;
        } else {
          _lastHitCornerId = newCorner;
        }
      }
      rebuildLayoutScene();
    } else if (_state == DrawingClutter) {

      _stroke.append(
          normalize(_options.camera().toSpace(gui::MakeCoreVec(e->pos()))));
      rebuildStrokeScene();

    } else if (_state == ConnectCoplanarFaces) {
      if (_lastHitFaceId != -1 && _faceClicked != -1) {
        _anno->setCoplanar(_lastHitFaceId, _faceClicked);
        _lastHitFaceId = _faceClicked = -1;
        rebuildLayoutScene();
      } else if (_lastHitFaceId == -1 && _faceClicked != -1) {
        _lastHitFaceId = _faceClicked;
      }
    }

    update();
  } else {
    QGLWidget::mousePressEvent(e);
  }
}

void PILayoutAnnotationWidget::mouseMoveEvent(QMouseEvent *e) {
  if (!_anno) {
    return;
  }
  QVector3D t(e->pos() - _lastPos);
  t.setX(-t.x());
  if (e->buttons() & Qt::MidButton) {
    _options.camera().moveCenterWithEyeFixed(MakeCoreVec(t));
    setCursor(Qt::ClosedHandCursor);
    update();
  } else if (e->buttons() & Qt::LeftButton) {
    if (_cornerClicked != -1) {
      _anno->corners[_cornerClicked] =
          normalize(_options.camera().toSpace(gui::MakeCoreVec(e->pos())));
      rebuildLayoutScene();
      update();
    }
  } else {
    QGLWidget::mouseMoveEvent(e);
  }
  _lastPos = e->pos();
}

void PILayoutAnnotationWidget::mouseReleaseEvent(QMouseEvent *e) {
  if (!_anno) {
    return;
  }
  unsetCursor();
  QGLWidget::mouseReleaseEvent(e);
}

void PILayoutAnnotationWidget::wheelEvent(QWheelEvent *e) {
  if (!_anno) {
    return;
  }
  _options.camera().setFocal(_options.camera().focal() *
                             exp(e->delta() / 1000.0));
  update();
  QGLWidget::wheelEvent(e);
}

void PILayoutAnnotationWidget::keyPressEvent(QKeyEvent *e) {
  if (!_anno) {
    return;
  }
  if (e->key() == Qt::Key_Space) {
    if (_state == DrawingClutter) {
      acceptClutter();
      rebuildCluttersScene();
    }
    clearStroke();
    update();
  } else if (e->key() == Qt::Key_Escape) {
    clearStroke();
    update();
  }
  QGLWidget::keyPressEvent(e);
}

void PILayoutAnnotationWidget::rebuildLayoutScene() {
  SceneBuilder sb;
  sb.installingOptions().lineWidth = 10.0;
  sb.installingOptions().pointSize = 30.0;

  auto &corners = _anno->corners;

  // corners
  _cornerPoints.resize(corners.size());
  for (int i = 0; i < corners.size(); i++) {
    _cornerPoints[i].component.component =
        normalize(corners[i]) * visualDepthBorder;
    _cornerPoints[i].component.color = gui::White;
    _cornerPoints[i].decoration = i;
  }
  sb.installingOptions().discretizeOptions.color(gui::White);
  sb.begin(_cornerPoints,
           [this](gui::InteractionID iid,
                  const core::Decorated<Colored<Point3>, int> &point) {
             std::cout << "corner " << point.decoration << " is clicked"
                       << std::endl;
             _cornerClicked = point.decoration;
             _borderClicked = -1;
             _faceClicked = -1;
           })
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPoints)
      .end();

  // borders
  _borderLines.clear();
  _borderLines.resize(_anno->border2corners.size());
  for (int i = 0; i < _anno->border2corners.size(); i++) {
    _borderLines[i].decoration = i;
    auto &cs = _anno->border2corners[i];
    _borderLines[i].component.component.first =
        normalize(corners[cs.first]) * visualDepthBorder;
    _borderLines[i].component.component.second =
        normalize(corners[cs.second]) * visualDepthBorder;
    _borderLines[i].component.color =
        _anno->border2connected[i] ? gui::White : gui::Black;
  }
  sb.begin(_borderLines,
           [this](gui::InteractionID iid,
                  const core::Decorated<Colored<Line3>, int> &line) {
             std::cout << "border " << line.decoration << " is clicked"
                       << std::endl;
             _cornerClicked = -1;
             _borderClicked = line.decoration;
             _faceClicked = -1;
           })
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XLines)
      .end();

  // faces
  _facePolygons.clear();
  // rebuild faces
  _facePolygons.resize(_anno->face2control.size());
  std::vector<Vec3> facePolygonCenters(_anno->face2control.size(), Origin());
  std::vector<Line3> facePolygonCenterLines;
  const static gui::ColorTable rgbtable = gui::RGB;
  for (int i = 0; i < _anno->face2control.size(); i++) {
    _facePolygons[i].decoration = i;
    auto &control = _anno->face2control[i];
    _facePolygons[i].component.color =
        control.dof() == 3
            ? gui::White
            : (control.dof() == 2
                   ? rgbtable[control.orientationNotClaz].blendWith(gui::Black,
                                                                    0.7)
                   : rgbtable[control.orientationClaz]);

    auto &polygon = _facePolygons[i].component.component;
    const auto &cs = _anno->face2corners[i];

    Vec3 &center = facePolygonCenters[i];
    for (int corner : cs) {
      polygon.corners.push_back(normalize(corners[corner]) * visualDepthFace);
      center += normalize(polygon.corners.back());
    }
    center = normalize(center) * visualDepthFaceCenter;
    polygon.normal = normalize(center);
    for (int corner : cs) {
      facePolygonCenterLines.push_back(
          normalize(Line3(center, corners[corner])) *
          visualDepthFaceCenterLines);
    }
  }
  sb.begin(_facePolygons,
           [this](gui::InteractionID iid,
                  const Decorated<Colored<Polygon3>, int> &polygon) {
             std::cout << "face " << polygon.decoration << " is clicked"
                       << std::endl;
             _cornerClicked = -1;
             _borderClicked = -1;
             _faceClicked = polygon.decoration;
             std::cout << "its corners include: ";
             for (int c : _anno->face2corners[_faceClicked]) {
               std::cout << c << " ";
             }
             std::cout << std::endl;
           })
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XPanorama)
      .end();
  sb.installingOptions().discretizeOptions.color(gui::White);
  // sb.begin(facePolygonCenters).pointSize(10.0).shaderSource(gui::OpenGLShaderSourceDescriptor::XPoints).end();
  // sb.begin(facePolygonCenterLines).lineWidth(1.0).shaderSource(gui::OpenGLShaderSourceDescriptor::XLines).end();

  // coplanarFacePair
  _coplanarFacePairLines.clear();
  _coplanarFacePairLines.resize(_anno->coplanarFacePairs.size());
  for (int i = 0; i < _anno->coplanarFacePairs.size(); i++) {
    auto &fp = _anno->coplanarFacePairs[i];
    _coplanarFacePairLines[i].component = normalize(
        Line3(facePolygonCenters[fp.first], facePolygonCenters[fp.second]) *
        visualDepthFaceCoplanarLine);
    _coplanarFacePairLines[i].decoration = i;
  }
  sb.installingOptions().discretizeOptions.color(gui::White);
  /* sb.begin(_coplanarFacePairLines, [this](gui::InteractionID iid, const
   Decorated<Line3, int> & line) {
       std::cout << "face " << _anno->coplanarFacePairs[line.decoration].first
           << " and face " << _anno->coplanarFacePairs[line.decoration].second
           << " are labeled as coplanar" << std::endl;
   }).lineWidth(2.0).shaderSource(gui::OpenGLShaderSourceDescriptor::XLines).end();*/

  _layoutScene = sb.scene();
}

void PILayoutAnnotationWidget::rebuildCluttersScene() {
  _clutterPolygons = decltype(_clutterPolygons)(_anno->clutters.size());
  for (int i = 0; i < _anno->clutters.size(); i++) {
    auto &poly = _clutterPolygons[i];
    poly.decoration = i;
    poly.component = _anno->clutters[i];
    for (auto &c : poly.component.corners) {
      c = normalize(c) * visualDepthClutter;
    }
  }
  SceneBuilder sb;
  sb.installingOptions().discretizeOptions.color(gui::Black);
  sb.begin(_clutterPolygons)
      .shaderSource(gui::OpenGLShaderSourceDescriptor::XTriangles)
      .end();

  _cluttersScene = sb.scene();
}

void PILayoutAnnotationWidget::rebuildStrokeScene() {
  SceneBuilder sb;
  sb.installingOptions().defaultShaderSource =
      gui::OpenGLShaderSourceDescriptor::XLines;
  sb.installingOptions().lineWidth = 20.0;
  sb.installingOptions().discretizeOptions.color(gui::Black);

  sb.add(_stroke);
  _strokeScene = sb.scene();
}

void PILayoutAnnotationWidget::acceptClutter() {
  if (_stroke.size() >= 3) {
    _anno->clutters.emplace_back(_stroke);
  }
}

void PILayoutAnnotationWidget::clearStroke() {
  _lastHitCornerId = -1;
  _lastHitFaceId = -1;
  _faceClicked = _cornerClicked = _borderClicked = -1;
  _stroke.clear();
  _strokeScene = gui::Scene();
}
}
}
