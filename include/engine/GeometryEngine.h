#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include "structures/Mesh.h"


class GeometryEngine : protected QOpenGLFunctions
{
public:
    GeometryEngine();

    virtual ~GeometryEngine();

    void draw(QOpenGLShaderProgram *program, const Mesh* mesh, GLenum drawMode);

private:
    void init();

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
};