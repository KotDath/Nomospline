#pragma once

#include <QString>
#include "structures/Mesh.h"

class MeshLoader {
public:
    Mesh* load(const QString& path);
};
