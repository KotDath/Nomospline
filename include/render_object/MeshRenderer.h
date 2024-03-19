#pragma once

#include "RenderObject.h"
#include "structures/Mesh.h"

class MeshRenderer : protected RenderObject {
public:
    void setMesh(const Mesh* mesh);
    void setDrawMode(GLenum mode);
    void initShaders() override;
    void paint() override;
private:
    const Mesh* mesh;
    GLenum drawMode = GL_TRIANGLES;
};