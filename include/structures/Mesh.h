#pragma once

#include <QVector>
#include <QVector3D>
#include <QOpenGLBuffer>

struct VertexData
{
    QVector3D position;
};

struct Mesh
{
    QVector<VertexData> vertices;
    QVector<GLushort> indices;
};