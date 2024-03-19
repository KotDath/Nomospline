#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include "engine/GeometryEngine.h"
#include "engine/Camera.h"
#include "structures/NURBS.h"
#include "structures/SplineUtils.h"


class OpenGLWindow : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
private:
    virtual ~OpenGLWindow();

protected:
    void initShaders();


    void initializeGL() override;

    void resizeGL(int w, int h) override;

    void paintGL() override;

    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

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
    void calculateIntersection();
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

    Mesh* intersectionPoints;

    QVector<QVector<QVector3D>> PointWithNormals;

    QPoint previousMousePosition;



    bool isPoint = false;
    bool isLines = false;
    bool isTriangles = false;
    bool isNormal = false;

    float normalSize = 1.0f;

    static QVector<QVector4D> filterPoints(const QVector<QVector4D>& points);
};