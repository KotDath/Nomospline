#pragma once

#include "RenderObject.h"
#include "engine/Camera.h"


class GizmosRenderer : protected RenderObject {
public:
    GizmosRenderer();
    void initShaders() override;
    void paint() override;
private:
    const GLfloat vao[36];
    const GLuint vbo[6];

    QMatrix4x4 perspective;
};