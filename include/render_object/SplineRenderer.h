#pragma once

#include "RenderObject.h"
#include "structures/NURBS.h"

class SplineRenderer : protected RenderObject {
public:
    void setSpline(const NURBS* spline);
    void initShaders() override;
    void paint() override;
private:
    QOpenGLShaderProgram* grid_program = nullptr;
    const NURBS* spl;
    GLenum drawMode = GL_POINTS;

    GLuint count = 0;
    GLuint spline_count = 0;
};