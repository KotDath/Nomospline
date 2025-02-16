#include "render_object/PointRenderer.h"

void PointRenderer::setMesh(const Mesh *mesh) {
    msh = mesh;
    arrayBuf.bind();
    qDebug() << "Count: " << msh->vertices.count();
    arrayBuf.allocate(msh->vertices.data(), msh->vertices.count() * sizeof(VertexData));

    indexBuf.bind();
    indexBuf.allocate(msh->indices.data(), msh->indices.count() * sizeof(GLuint));
}

void PointRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/points/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/points/fshader.glsl");

        program->link();
    }
}

void PointRenderer::paint() {
    program->bind();
    arrayBuf.bind();
    indexBuf.bind();

    int mvp_matrix = program->uniformLocation("mvp_matrix");

    if (mvp_matrix != -1) {

        program->setUniformValue(mvp_matrix,
                                 sceneData->camera.getProjection() * sceneData->camera.getView() * model);
    }

    int point_color = program->uniformLocation("point_color");

    if (point_color != -1) {

        program->setUniformValue(point_color, 1.0, 0.0, 0.0);
    }


    int point_size = program->uniformLocation("point_size");

    if (point_size != -1) {

        program->setUniformValue(point_size, 5.0f);
    }


    int vertexLocation = program->attributeLocation("a_Position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));
    glDrawArrays(GL_POINTS, 0, msh->vertices.count());

    program->release();

    arrayBuf.release();
    indexBuf.release();
}
