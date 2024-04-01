#pragma once

#include <QtQuick/QQuickItem>
#include <QtQuick/QQuickWindow>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QVector>

#include "engine/Camera.h"
#include "render_object/MeshRenderer.h"
#include "render_object/BackgroundRenderer.h"
#include "engine/SceneData.h"
#include "render_object/GizmosRenderer.h"
#include "render_object/SplineRenderer.h"
#include "render_object/PointRenderer.h"
#include "render_object/NormalRenderer.h"


//! [1]
class SceneRenderer : public QObject, protected QOpenGLFunctions {
Q_OBJECT

public:
    SceneRenderer();

    ~SceneRenderer();

    void setViewportSize(const QSize &size) { m_viewportSize = size; }

    void setPosition(const QSize &size) { m_position = size; }

    void setWindow(QQuickWindow *window) { m_window = window; }

    void resize();

public slots:

    void init();

    void paint();

    void initShaders();

    void mouseMoveEvent(QMouseEvent *event);

    void mousePressEvent(QMouseEvent *event);

    void wheelEvent(QWheelEvent *event);

private:


    QSize m_viewportSize;
    QSize m_position;
    QOpenGLShaderProgram *m_program = nullptr;
    QQuickWindow *m_window = nullptr;

    SceneData *sceneData = nullptr;
    MeshRenderer meshRenderer;
    BackgroundRenderer backgroundRenderer;
    SplineRenderer splineRenderer;
    GizmosRenderer gizmosRenderer;
    PointRenderer pointRenderer;
    NormalRenderer normalRenderer;

    QPoint previousMousePosition;
};

class Scene : public QQuickItem {
Q_OBJECT


    QML_ELEMENT

public:
    Scene();

public:
    void addGeometry(Mesh *mesh);

    void addSpline(NURBS *pNurbs);

    virtual void mouseMoveEvent(QMouseEvent *event);

    virtual void mousePressEvent(QMouseEvent *event);

    virtual void wheelEvent(QWheelEvent *event);

public slots:

    void sync();

    void cleanup();

private slots:

    void handleWindowChanged(QQuickWindow *win);

private:
    void releaseResources() override;

    SceneRenderer *m_renderer;
};