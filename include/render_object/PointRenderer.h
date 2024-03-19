#pragma once

#include "RenderObject.h"
#include "structures/Mesh.h"

class PointRenderer : protected RenderObject {
public:
    void setMesh(const Mesh* mesh);
    void initShaders() override;
    void paint() override;
private:
    const Mesh* msh;
    GLenum drawMode = GL_POINTS;
};