
#include "render_object/BackgroundRenderer.h"

void BackgroundRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/background/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/background/fshader.glsl");
        program->link();
    }
}

void BackgroundRenderer::paint() {
    program->bind();
    glDisable(GL_DEPTH_TEST);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    program->enableAttributeArray(0);
    program->setAttributeArray(0, GL_FLOAT, values, 2);
    program->setUniformValue("color2", 0.453125, 0.57421875, 0.875);
    program->setUniformValue("color1", 1, 1, 1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glEnable(GL_DEPTH_TEST);

    program->release();
}

BackgroundRenderer::BackgroundRenderer() : values{
        -1, -1,
        1, -1,
        -1, 1,
        1, 1
}, RenderObject() {


}
