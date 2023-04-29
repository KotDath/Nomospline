#pragma once

#include <QVector4D>
#include <QOpenGLBuffer>

#include "Mesh.h"

struct NURBS {

    Mesh* evaluate();

    QVector4D* controlPoints; // (x, y, z, w где w - вес)
    GLsizei controlPointsCount;
    GLfloat* knotU;
    GLsizei uSize;
    GLfloat* knotV;
    GLsizei vSize;

};
