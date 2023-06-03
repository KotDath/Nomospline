#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "engine/GeometryEngine.h"
#include "engine/Camera.h"
#include "structures/NURBS.h"


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
    void addSpline(NURBS *newSpline);
    void clear();
    void evaluateSplines();
    void setNormalLength(float size);

public slots:
    void setPoint(bool point);
    void setLine(bool line);
    void setTriangle(bool triangle);
    void setNormal(bool normal);
private:
    QOpenGLShaderProgram meshProgram;
    QOpenGLShaderProgram pointsProgram;
    QOpenGLShaderProgram linesProgram;
    GeometryEngine *engine;
    Camera camera;
    QVector<Mesh*> meshes;
    QVector<Mesh*> splineMeshes;
    QVector<Mesh*> normalMeshes;
    QVector<NURBS*> splines;
    QMatrix4x4 model;

    QVector<QVector<QVector3D>> PointWithNormals;

    QPoint previousMousePosition;



    bool isPoint = false;
    bool isLines = false;
    bool isTriangles = false;
    bool isNormal = false;

    float normalSize = 1.0f;
};