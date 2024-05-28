#pragma once

#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>

struct NURBSCurve {

    QVector<QVector3D> controlPoints; // (u, v, w)
    QVector <GLfloat> knott;

    GLsizei tDegree;

    bool isLooped = false;
};
