#pragma once

#include <QtOpenGL>
#include <QtWidgets>
#include <functional>

#include "scene.hpp"
#include "pi_graph_annotation.hpp"

namespace pano {
namespace experimental {

class PILayoutAnnotationWidget : public QGLWidget {
public:
  PILayoutAnnotationWidget(QWidget *parent = nullptr);
  ~PILayoutAnnotationWidget();

  void setCurAnnotation(PILayoutAnnotation *anno, const QString *imagePath);

protected:
  virtual void paintEvent(QPaintEvent *e) override;
  virtual void mousePressEvent(QMouseEvent *e) override;
  virtual void mouseMoveEvent(QMouseEvent *e) override;
  virtual void mouseReleaseEvent(QMouseEvent *e) override;
  virtual void wheelEvent(QWheelEvent *e) override;
  virtual void keyPressEvent(QKeyEvent *e) override;

private:
  void clearStroke();

  void rebuildLayoutScene();
  void rebuildCluttersScene();
  void rebuildStrokeScene();

  void acceptClutter();

private:
  QPoint _lastPos;

  gui::Scene _imageScene;

  gui::Scene _layoutScene;
  std::vector<Decorated<gui::Colored<Point3>, int>> _cornerPoints;
  std::vector<Decorated<gui::Colored<Line3>, int>> _borderLines;
  std::vector<Decorated<gui::Colored<Polygon3>, int>> _facePolygons;
  std::vector<Decorated<Line3, int>> _coplanarFacePairLines;

  gui::Scene _cluttersScene;
  std::vector<Decorated<Polygon3, int>> _clutterPolygons;

  gui::Scene _vpsScene;
  gui::Scene _strokeScene;

  gui::RenderOptions _options;
  PILayoutAnnotation *_anno;
  const QString *_imagePath;

  enum State { Pick, DrawingBorder, DrawingClutter, ConnectCoplanarFaces };
  State _state;
  QAction *_actPick;
  QAction *_actDrawBorder;
  QAction *_actDrawClutter;
  QAction *_actConnectCoplanarFaces;

  Chain3 _stroke;

  int _cornerClicked, _borderClicked, _faceClicked;

  int _lastHitCornerId;
  int _lastHitFaceId;

  bool _showLayouts;
  bool _showClutters;
  bool _showVPs;
};
}
}
