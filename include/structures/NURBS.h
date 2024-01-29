#pragma once

#include <QVector3D>
#include <QVector4D>
#include <QOpenGLBuffer>
#include <QGenericMatrix>

#include "Mesh.h"

struct NURBS
{

    Mesh *evaluate();



    void init();

    QVector<QVector<QVector4D>> controlPoints; // (x, y, z, w)
    QVector<GLfloat> knotU;
    QVector<GLfloat> knotV;

    GLsizei uDegree;
    GLsizei vDegree;

    QVector3D getPoint(GLfloat u, GLfloat v);

    std::pair<QVector3D, QVector3D> getDerivatives(GLfloat u, GLfloat v);

    std::pair<QVector3D, QVector3D> getDerivatives2(GLfloat u, GLfloat v);

    QVector3D getDerivativesMixed(GLfloat u, GLfloat v);

    GLfloat getAdaptiveStepU(GLfloat u, GLfloat v);

    GLfloat getAdaptiveStepV(GLfloat u, GLfloat v);

    GLfloat getAdaptiveStepCurveU(GLfloat u, GLfloat v);

    GLfloat getAdaptiveStepCurveV(GLfloat u, GLfloat v);

    QVector3D getNormal(GLfloat u, GLfloat v);

    QVector<QVector4D> getInitialPoints(NURBS *otherSpline);

    QVector<QVector3D> iterPointsPlus(NURBS *otherSpline, const QVector4D &initialPoint);
    QVector<QVector3D> iterPointsMinus(NURBS *otherSpline, const QVector4D &initialPoint);

private:
    int findSpan(GLsizei degree, const QVector<GLfloat> &knots, GLfloat param);

    std::vector<GLfloat> basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat> &knots, GLfloat u);

    static GLfloat getStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                                 const QVector3D &t);

    static int getDirectionStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                                      const QVector3D &t);

    QVector<QVector4D> curveToSurfaceIntersectionU(GLfloat u, NURBS *otherSpline);

    QVector<QVector4D> curveToSurfaceIntersectionV(GLfloat v, NURBS *otherSpline);

    static QMatrix3x3
    getJacobian3D(NURBS *spline1, NURBS *spline2, int constIndex, GLfloat constValue, const QVector3D &current);

    static QMatrix4x4
    getJacobian4D(NURBS *spline1, NURBS *spline2, const QVector4D &current);

    static QVector3D
    NewtonSolution3D(NURBS *spline1, NURBS *spline2, const QVector3D &zeroSolution, int constIndex, GLfloat constValue);

    static QVector4D
    NewtonSolution4D(NURBS *spline1, NURBS *spline2, const QVector4D &zeroSolution);

    bool isBound(NURBS* otherSpline, const QVector4D& point);

};
