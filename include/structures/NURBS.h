#pragma once

#include <QVector3D>
#include <QVector4D>
#include <QOpenGLBuffer>

#include "Mesh.h"

struct NURBS {

    Mesh* evaluate();
    QVector<QVector3D> intersectionLine(NURBS* otherSpline);
    void init();

    QVector<QVector<QVector4D>> controlPoints; // (x, y, z, w)
    QVector<GLfloat> knotU;
    QVector<GLfloat> knotV;

    GLsizei uDegree;
    GLsizei vDegree;

    QVector3D getPoint(GLfloat u, GLfloat v);
    std::pair<QVector3D, QVector3D> getDerivatives(GLfloat u, GLfloat v);
    std::pair<QVector3D, QVector3D> getDerivatives2(GLfloat u, GLfloat v);
    QVector3D getNormal(GLfloat u, GLfloat v);

private:
    int findSpan(GLsizei degree, const QVector<GLfloat>& knots, GLfloat param);
    std::vector<GLfloat> basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat>& knots, GLfloat u);

    QVector<QVector2D> getInitialPoints(NURBS* otherSpline);

};
