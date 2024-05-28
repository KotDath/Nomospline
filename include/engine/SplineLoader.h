#pragma once

#include <QString>
#include "structures/NURBS.h"
#include "structures/NURBSCurve.h"

class SplineLoader {
public:
    NURBS* load(const QString &path);
    static NURBSCurve* loadCurve(const QString &path);
};
