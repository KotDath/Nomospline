#pragma once

#include <QVector3D>
#include <QOpenGLBuffer>

struct VertexData
{
    QVector3D position;
};

struct Mesh
{
    VertexData* vertices;
    GLsizei verticesCount;
    GLushort* indices;
    GLsizei indicesCount;
};