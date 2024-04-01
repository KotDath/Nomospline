#pragma once

#include "RenderObject.h"
#include "structures/Mesh.h"

class NormalRenderer : protected RenderObject {
public:
    void setMesh(const Mesh* mesh);
    void initShaders() override;
    void paint() override;
private:
    const Mesh* msh;
};