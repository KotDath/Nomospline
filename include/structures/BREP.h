#pragma once

#include "NURBS.h"
#include "PolylineCurve.h"
#include "SplineUtils.h"

struct BREP {
    bool isInsideBREP(QVector3D p);
    QMap<NURBS*, QVector<PolylineCurve*>> surfaces;
};

