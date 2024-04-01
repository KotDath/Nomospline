//
// Created by kotdath on 3/19/24.
//
#include "render_object/NormalRenderer.h"

void NormalRenderer::setMesh(const Mesh *mesh) {
    msh = mesh;
    arrayBuf.bind();
    arrayBuf.allocate(msh->vertices.data(), msh->vertices.count() * sizeof(VertexData));

    indexBuf.bind();
    indexBuf.allocate(msh->indices.data(), msh->indices.count() * sizeof(GLuint));
}

void NormalRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/normals/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Geometry, ":/shaders/normals/gshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/normals/fshader.glsl");

        program->link();
    }
}

void NormalRenderer::paint() {
    program->bind();
    arrayBuf.bind();
    indexBuf.bind();

    int mvp_matrix = program->uniformLocation("gxl3d_ModelViewProjectionMatrix");

    if (mvp_matrix != -1) {

        program->setUniformValue(mvp_matrix,
                                 sceneData->camera.getProjection() * sceneData->camera.getView() * model);
    }

    int normal_length = program->uniformLocation("normal_length");

    if (normal_length != -1) {

        program->setUniformValue(normal_length, 1.0f);
    }

    int vertexLocation = program->attributeLocation("a_Position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));

    quintptr offset = 0;

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    int normalLocation = program->attributeLocation("a_Norm");
    program->enableAttributeArray(normalLocation);
    program->setAttributeBuffer(normalLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    glDrawElements(GL_TRIANGLES, msh->indices.count(), GL_UNSIGNED_INT, nullptr);

    program->release();

    arrayBuf.release();
    indexBuf.release();
}

