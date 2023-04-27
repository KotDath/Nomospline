#pragma once

#include <QVector4D>
#include <QOpenGLBuffer>

struct NURBS {
    QVector4D* controlPoints; // (x, y, z, w где w - вес)
    GLsizei controlPointsCount;
    GLfloat* knotU;
    GLsizei uSize;
    GLfloat* knotV;
    GLsizei vSize;

};
