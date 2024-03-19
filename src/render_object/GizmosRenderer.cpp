//
// Created by kotdath on 3/13/24.
//
#include "render_object/GizmosRenderer.h"

GizmosRenderer::GizmosRenderer() : vao{
        0, 0, 0, 1, 0, 0,
        1, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
        0, 0, 1, 0,0, 1
}, vbo{
        0, 1,
        2, 3,
        4, 5
} {
    arrayBuf.bind();
    arrayBuf.allocate(vao, sizeof(GLfloat) * 36);

    indexBuf.bind();
    indexBuf.allocate(vbo, sizeof(GLuint) * 6);
    const float factor = 2.0;

    const qreal zNear = 0.1, zFar = 10000.0, fov = 60.0;

    // Set perspective projection
    perspective.perspective(fov, 1, zNear, zFar);

}

void GizmosRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/gizmos/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/gizmos/fshader.glsl");

        program->link();
    }
}

void GizmosRenderer::paint() {
    program->bind();
    arrayBuf.bind();
    indexBuf.bind();

    glLineWidth(5);
    int mvp_matrix = program->uniformLocation("mvp_matrix");

    const float factor = 2.5;
    auto camera = SceneData::getInstance()->camera;
    auto pos = camera.getPosition().normalized();


    camera.setPosition(pos.x() * factor, pos.y() * factor, pos.z() * factor);

    if (mvp_matrix != -1) {

        program->setUniformValue(mvp_matrix,  perspective * camera.getView() * model);
    }

    int vertexLocation = program->attributeLocation("a_Position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(GLfloat) * 6);
    int colorLocation = program->attributeLocation("a_Color");
    program->enableAttributeArray(colorLocation);
    program->setAttributeBuffer(colorLocation, GL_FLOAT, sizeof(GLfloat) * 3, 3, sizeof(GLfloat) * 6);
    glDrawArrays(GL_LINES, 0, 18);

    arrayBuf.release();
    indexBuf.release();
    program->release();
}


