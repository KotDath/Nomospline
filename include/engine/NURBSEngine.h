#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>

#include "structures/NURBS.h"

class NURBSEngine: protected QOpenGLFunctions
{
public:
    NURBSEngine();

    virtual ~NURBSEngine();

    void draw(QOpenGLShaderProgram *program, const NURBS* spline, GLenum drawMode);

private:
    void init();

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
};