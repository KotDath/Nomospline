#include "structures/BREP.h"

bool BREP::isInsideBREP(QVector3D p) {
    QVector<QVector2D> intersections;
    QVector2D minPoint;
    auto minDistance = std::numeric_limits<float>::max();
    NURBS* nearestSpline = nullptr;
    for (const auto& spl : surfaces.keys()) {
        auto point = SplineUtils::PointProjection(spl, p);
        auto distance = (p - SplineUtils::getPoint(spl, point.x(), point.y())).length();
        if (distance < minDistance && SplineUtils::isBound(spl, point)) {
            minPoint = point;
            minDistance = distance;
            nearestSpline = spl;
        }
    }

    bool isInside = false;

    if (minDistance < std::numeric_limits<float>::max()) {
        auto n = SplineUtils::getNormal(nearestSpline, minPoint.x(), minPoint.y());
        auto w = p - SplineUtils::getPoint(nearestSpline, minPoint.x(), minPoint.y());

        qDebug() << n << w << QVector3D::dotProduct(n, w);
        bool isInside = QVector3D::dotProduct(n, w) < 0;

        qDebug() << "AAAAAAA " << minPoint << minDistance << isInside;
        return isInside;
    } else {
        qDebug() << "Whoops";
        return false;
    }


}
