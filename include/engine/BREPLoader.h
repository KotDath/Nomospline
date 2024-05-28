#pragma once

#include "structures/BREP.h"
#include "SplineLoader.h"

class BREPLoader {
public:
    BREP* load(const QString& path);
private:
    SplineLoader splineLoader;
};
