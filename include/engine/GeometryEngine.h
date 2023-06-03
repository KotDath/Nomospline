#pragma once

#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>

#include "structures/Mesh.h"
#include "structures/NURBS.h"



class GeometryEngine : protected QOpenGLFunctions_4_5_Core
{
public:
    GeometryEngine();

    virtual ~GeometryEngine();

    void draw(QOpenGLShaderProgram *program, const Mesh* mesh, GLenum drawMode);
    void draw(QOpenGLShaderProgram *program, const NURBS* nurbs);

private:
    void init();

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
};