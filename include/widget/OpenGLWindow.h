#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "engine/GeometryEngine.h"
#include "engine/Camera.h"


class OpenGLWindow : public QOpenGLWidget, protected QOpenGLFunctions
{
protected:
    void initShaders();


    void initializeGL() override;

    void resizeGL(int w, int h) override;

    void paintGL() override;

public:
    void setGeometry(Mesh* newMesh) {mesh = newMesh;}

private:
    QOpenGLShaderProgram program;
    GeometryEngine* engine;
    Camera camera;
    Mesh* mesh;
    QMatrix4x4 model;

};