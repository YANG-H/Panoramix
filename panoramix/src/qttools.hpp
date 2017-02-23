#pragma once

#include <QtCore>
#include <QtGui>

#include <Eigen/Core>

#include "basic_types.hpp"
#include "color.hpp"

namespace pano {
namespace gui {

// color
inline QRgb MakeQRgb(const Color &c) {
  return qRgba(c.red(), c.green(), c.blue(), c.alpha());
}

inline QColor MakeQColor(const Color &c) { return QColor(MakeQRgb(c)); }

// vector
template <class T> inline QVector2D MakeQVec(const core::Vec<T, 2> &v) {
  return QVector2D(static_cast<float>(v[0]), static_cast<float>(v[1]));
}

template <class T> inline QVector3D MakeQVec(const core::Vec<T, 3> &v) {
  return QVector3D(static_cast<float>(v[0]), static_cast<float>(v[1]),
                   static_cast<float>(v[2]));
}

inline core::Vec2 MakeCoreVec(const QPoint &p) {
  return core::Vec2(p.x(), p.y());
}
inline core::Vec2 MakeCoreVec(const QPointF &p) {
  return core::Vec2(p.x(), p.y());
}

inline core::Vec3 MakeCoreVec(const QVector3D &v) {
  return core::Vec3(v.x(), v.y(), v.z());
}
inline core::Vec4 MakeCoreVec(const QVector4D &v) {
  return core::Vec4(v.x(), v.y(), v.z(), v.w());
}

template <class T> inline QVector4D MakeQVec(const core::Vec<T, 4> &v) {
  return QVector4D(static_cast<float>(v[0]), static_cast<float>(v[1]),
                   static_cast<float>(v[2]), static_cast<float>(v[3]));
}

inline QVector4D MakeQVec(const Color &v) {
  return QVector4D(v.redf(), v.greenf(), v.bluef(), v.alphaf());
}

// matrix
template <class T, int M, int N>
QGenericMatrix<N, M, T> MakeQMatrix(const core::Mat<T, M, N> &m) {
  QGenericMatrix<N, M, T> mat;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      mat(i, j) = m(i, j);
    }
  }
  return mat;
}
QMatrix4x4 MakeQMatrix(const core::Mat<float, 4, 4> &m);
QMatrix4x4 MakeQMatrix(const core::Mat<double, 4, 4> &m);

template <class T, int M, int N>
core::Mat<T, M, N> MakeCoreMatrix(const QGenericMatrix<N, M, T> &m) {
  core::Mat<T, M, N> mat;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      mat(i, j) = m(i, j);
    }
  }
  return mat;
}
core::Mat<float, 4, 4> MakeCoreMatrix(const QMatrix4x4 &m);

// point
template <class T> inline QPointF MakeQPointF(const core::Point<T, 2> &p) {
  return QPointF(static_cast<float>(p[0]), static_cast<float>(p[1]));
}
inline core::Point<qreal, 2> MakeCorePoint(const QPointF &p) {
  return core::Point<qreal, 2>(p.x(), p.y());
}
inline QPoint MakeQPoint(const core::Pixel &p) { return QPoint(p.x, p.y); }

// size
inline QSizeF MakeQSizeF(const core::Size &sz) {
  return QSizeF(sz.width, sz.height);
}
inline QSize MakeQSize(const core::Sizei &sz) {
  return QSize(sz.width, sz.height);
}
inline core::Sizei MakeCoreSize(const QSize & sz) {
    return core::Sizei(sz.width(), sz.height());
}

// lines
template <class T>
inline QLineF MakeQLineF(const core::Line<Point<T, 2>> &line) {
  return QLineF(MakeQPointF(line.first), MakeQPointF(line.second));
}
inline core::Line<Point<qreal, 2>> MakeCoreLine(const QLineF &line) {
  return core::Line<Point<qreal, 2>>(MakeCorePoint(line.p1()),
                                     MakeCorePoint(line.p2()));
}

// image
QImage MakeQImage(const core::Image &im);
inline QPixmap MakeQPixmap(const core::Image &im) {
  return QPixmap::fromImage(MakeQImage(im));
}

core::Image MakeCVMat(const QImage &im, bool inCloneImageData = true);
inline core::Image MakeCVMat(const QPixmap &im, bool inCloneImageData = true) {
  return MakeCVMat(im.toImage(), inCloneImageData);
}

// predefined widgets
template <class BaseWidgetT> class UIWidget : public BaseWidgetT {
public:
  explicit UIWidget(const std::vector<PenConfig> &pc, QWidget *parent = nullptr)
      : BaseWidgetT(parent), _activePenId(-1), _mode(Other),
        _penCursor(QPixmap(tr(":/icons/pencil_icon&16.png")), 0, 16) {
    setMouseTracking(true);
    setupPens(pc);
  }

private:
  void setupPens(const std::vector<PenConfig> &pc) {
    _pens.resize(pc.size());

    // setup pen selection actions
    setContextMenuPolicy(Qt::ActionsContextMenu);
    QActionGroup *bas = new QActionGroup(this);

    QAction *defaultAction = nullptr;
    connect(defaultAction = bas->addAction(tr("Pick")), &QAction::triggered,
            [this]() { _activePenId = -1; });
    connect(defaultAction, SIGNAL(triggered()), this, SLOT(updateCursor()));
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
      connect(action, SIGNAL(triggered()), this, SLOT(updateCursor()));
    }
    for (auto a : bas->actions())
      a->setCheckable(true);

    bas->setExclusive(true);
    defaultAction->setChecked(true);

    addActions(bas->actions());
  }

private:
  Q_SLOT void updateCursor() {
    if (_activePenId == -1) {
      unsetCursor();
    } else {
      setCursor(_penCursor);
    }
  }

protected:
  void mousePressEvent(QMouseEvent *e) override {
    if (e->buttons() & Qt::LeftButton) {
      _mode = ClickMode;
      _lastPoint = e->pos();
    } else {
      _mode = Other;
      QWidget::mousePressEvent(e);
    }
  }

  void mouseMoveEvent(QMouseEvent *e) override {
    if (e->buttons() & Qt::LeftButton) {
      _mode = DragMode;
      if (activePenId() != -1)
        dragContinue(MakeCoreVec(e->pos()));
      _lastPoint = e->pos();
    } else {
      QWidget::mousePressEvent(e);
    }
  }

  void mouseReleaseEvent(QMouseEvent *e) override {
    if (activePenId() != -1) {
      if (_mode == ClickMode) {
        clicked(MakeCoreVec(e->pos()));
      } else if (_mode == DragMode) {
        dragEnd(MakeCoreVec(e->pos()));
      }
    }
    _mode = Other;
    unsetCursor();
    QWidget::mouseReleaseEvent(e);
  }

protected:
  virtual void clicked(const core::Point2 &p) {}
  virtual void dragContinue(const core::Point2 &p) {}
  virtual void dragEnd(const core::Point2 &p) {}

public:
  int activePenId() const { return _activePenId; }
  core::Point2 lastPoint() const { return MakeCoreVec(_lastPoint); }
  QPen activePen() const {
    return _activePenId == -1 ? QPen() : _pens[_activePenId];
  }
  const QPen &pen(int id) const { return _pens[id]; }

private:
  QPointF _lastPoint;
  int _activePenId;
  std::vector<QPen> _pens;
  std::vector<PenConfig> _penConfigs;
  enum { DragMode, ClickMode, Other } _mode;
  QCursor _penCursor;
};
}
}
