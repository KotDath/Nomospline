#pragma once

#include "Mesh.h"

class FileLoader {
public:
    // pure virtual function
    virtual Mesh* load(const QString& path) = 0;
};