#pragma once

#include "structures/Mesh.h"
#include "structures/NURBS.h"


class BREP;

class Tesselator {
public:
    static Mesh *tesselate(NURBS *spl, QVector<QVector<QVector2D>> limits, int offset = 0, bool ignoreTrim = true);

    static void intersection(BREP* b1, BREP* b2);
    static void diff(BREP* b1, BREP* b2);

private:
    static float crossProduct(const QVector2D &p, const QVector2D &P1, const QVector2D &P2);

    static std::map<std::pair<float, float>, QVector<std::pair<QVector2D, QVector2D>>>
    clasterize(NURBS *spl, QVector<QVector<QVector2D>> limits);
    static bool RayToLineIntersection(const QVector2D& sourcePoint, const QVector2D& p1, const QVector2D& p2);
};
