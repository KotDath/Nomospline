#pragma once

#include "NURBS.h"


class SplineUtils {
public:

    static std::pair<QVector3D, QVector3D> getBoundingBox(NURBS* spline);

    static bool isBBIntersected(NURBS* s1, NURBS* s2);

    static Mesh *evaluate(NURBS *spline);

    static QVector3D getPoint(NURBS *spline, GLfloat u, GLfloat v);

    static std::pair<QVector3D, QVector3D> getDerivatives(NURBS *spline, GLfloat u, GLfloat v);

    static std::pair<QVector3D, QVector3D> getDerivatives2(NURBS *spline, GLfloat u, GLfloat v);

    static QVector3D getDerivativesMixed(NURBS *spline, GLfloat u, GLfloat v);

    static GLfloat getAdaptiveStepU(NURBS *spline, GLfloat u, GLfloat v);

    static GLfloat getAdaptiveStepV(NURBS *spline, GLfloat u, GLfloat v);

    static GLfloat getAdaptiveStepCurveU(NURBS *spline, GLfloat u, GLfloat v);

    static GLfloat getAdaptiveStepCurveV(NURBS *spline, GLfloat u, GLfloat v);

    static QVector3D getNormal(NURBS *spline, GLfloat u, GLfloat v);

    static QVector<QVector4D> getInitialPoints(NURBS *spline1, NURBS *spline2);

    static QVector<QVector3D> iterPointsPlus(NURBS *spline1, NURBS *spline2, const QVector4D &initialPoint);
    static QVector<QVector3D> iterPointsMinus(NURBS *spline1, NURBS *spline2, const QVector4D &initialPoint);

private:
    static int findSpan(GLsizei degree, const QVector<GLfloat> &knots, GLfloat param);

    static std::vector<GLfloat> basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat> &knots, GLfloat u);

    static GLfloat getStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                                 const QVector3D &t);

    static int getDirectionStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                                      const QVector3D &t);

    static QVector<QVector4D> curveToSurfaceIntersectionU(NURBS * spline1, GLfloat u, NURBS *spline2);

    static QVector<QVector4D> curveToSurfaceIntersectionV(NURBS *spline1, GLfloat v, NURBS *spline2);

    static QMatrix3x3
    getJacobian3D(NURBS *spline1, NURBS *spline2, int constIndex, GLfloat constValue, const QVector3D &current);

    static QMatrix4x4
    getJacobian4D(NURBS *spline1, NURBS *spline2, const QVector4D &current);

    static QVector3D
    NewtonSolution3D(NURBS *spline1, NURBS *spline2, const QVector3D &zeroSolution, int constIndex, GLfloat constValue);

    static QVector4D
    NewtonSolution4D(NURBS *spline1, NURBS *spline2, const QVector4D &zeroSolution);

    static bool isBound(NURBS* spline1, NURBS* spline2, const QVector4D& point);

};
