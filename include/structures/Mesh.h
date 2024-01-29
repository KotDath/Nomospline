#pragma once

#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>

#include "timer.h"

struct VertexData
{
    VertexData(const QVector3D &vector3D, const QVector3D &vector3D1)
    {
        position = vector3D;
        normal = vector3D1;
    }

    VertexData(const QVector3D &vector3D)
    {
        position = vector3D;
    }

    QVector3D position;
    QVector3D normal;
};

struct Mesh
{
    QVector<VertexData> vertices;
    QVector<GLushort> indices;
};
