#pragma once

#include <QVector4D>
#include <QOpenGLBuffer>

#include "Mesh.h"

struct NURBS {

    Mesh* evaluate();
    void init();

    QVector<QVector3D> controlPoints; // (x, y, z)
    QVector<GLfloat> knotU;
    QVector<GLfloat> knotV;

    GLsizei uDegree;
    GLsizei vDegree;


private:
    int findSpan(GLsizei degree, const QVector<GLfloat>& knots, GLfloat param);
    std::vector<GLfloat> basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat>& knots, GLfloat u);


};
