#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "engine/GeometryEngine.h"
#include "engine/Camera.h"


class OpenGLWindow : public QOpenGLWidget, protected QOpenGLFunctions
{
private:
    virtual ~OpenGLWindow();

protected:
    void initShaders();


    void initializeGL() override;

    void resizeGL(int w, int h) override;

    void paintGL() override;

    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

public:
    void addGeometry(Mesh *newMesh);
    void clear();

private:
    QOpenGLShaderProgram program;
    GeometryEngine *engine;
    Camera camera;
    QVector<Mesh*> meshes;
    QMatrix4x4 model;

    QPoint previousMousePosition;
};