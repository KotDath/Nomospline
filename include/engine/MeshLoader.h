#pragma once

#include <QString>
#include "FileLoader.h"

class MeshLoader: public FileLoader {
public:
    Mesh* load(const QString& path) override;
};
