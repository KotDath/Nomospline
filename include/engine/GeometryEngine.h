#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include "Mesh.h"


class GeometryEngine : protected QOpenGLFunctions
{
public:
    GeometryEngine();

    virtual ~GeometryEngine();

    void draw(QOpenGLShaderProgram *program, const Mesh* mesh);

private:
    void init();

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
};