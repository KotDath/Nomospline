#pragma once

#include <QString>
#include "structures/NURBS.h"

class SplineLoader {
public:
    NURBS* load(const QString &path);
};
