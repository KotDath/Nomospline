#pragma once

#include "NURBS.h"
#include "engine/Tesselator.h"
#include "NURBSCurve.h"
#include "PolylineCurve.h"


class SplineUtils {
public:

    static QVector2D PointProjection(NURBS *spline, const QVector3D &origin);

    static std::pair<QVector3D, QVector3D> getBoundingBox(NURBS *spline);

    static bool isBBIntersected(NURBS *s1, NURBS *s2);

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

    static QVector<QVector4D> iterPointsPlus(NURBS *spline1, NURBS *spline2, int& pointIndex, QVector<QVector4D>& initialPoints);

    static QVector<QVector4D> iterPointsMinus(NURBS *spline1, NURBS *spline22, int& pointIndex, QVector<QVector4D>& initialPoints);

    static bool isBound(NURBS *spline, const QVector2D &point);

    static GLfloat getAdaptiveStepT(NURBSCurve *spl, GLfloat t);

    static QVector2D getPoint2D(NURBSCurve *spl, GLfloat t);

    static PolylineCurve* fitCurve(NURBSCurve * spl);

    static QVector4D fitPointBound(NURBS *spline1, NURBS *spline2, const QVector4D &point);

    static QVector2D fitPointBound(NURBS *spline1, const QVector2D &point);

    static void rotate(BREP* b, const QVector3D& xyz_rotate);

private:

    static QVector4D rotate_x(const QVector4D& point, float angle);
    static QVector4D rotate_y(const QVector4D& point, float angle);
    static QVector4D rotate_z(const QVector4D& point, float angle);

    static int findSpan(GLsizei degree, const QVector<GLfloat> &knots, GLfloat param);

    static std::vector<GLfloat> basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat> &knots, GLfloat u);

    static GLfloat getStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                                 const QVector3D &t);

    static int
    getDirectionStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                           const QVector3D &t);

    static QVector<QVector4D> curveToSurfaceIntersectionU(NURBS *spline1, GLfloat u, NURBS *spline2);

    static QVector<QVector4D> curveToSurfaceIntersectionV(NURBS *spline1, GLfloat v, NURBS *spline2);

    static QMatrix3x3
    getJacobian3D(NURBS *spline1, NURBS *spline2, int constIndex, GLfloat constValue, const QVector3D &current);

    static QMatrix4x4
    getJacobian4D(NURBS *spline1, NURBS *spline2, const QVector4D &current);

    static QVector3D
    NewtonSolution3D(NURBS *spline1, NURBS *spline2, const QVector3D &zeroSolution, int constIndex, GLfloat constValue);

    static QVector4D
    NewtonSolution4D(NURBS *spline1, NURBS *spline2, const QVector4D &zeroSolution);

    static bool isBound(NURBS *spline1, NURBS *spline2, const QVector4D &point);

};
