#pragma once

#include <QVector3D>
#include <QVector4D>
#include <QOpenGLBuffer>
#include <QGenericMatrix>

#include "Mesh.h"

struct NURBS
{

    QVector<QVector<QVector4D>> controlPoints; // (x, y, z, w)
    QVector<GLfloat> knotU;
    QVector<GLfloat> knotV;

    GLsizei uDegree;
    GLsizei vDegree;

};
