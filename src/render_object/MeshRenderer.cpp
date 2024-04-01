//
// Created by kotdath on 3/13/24.
//
#include "render_object/MeshRenderer.h"
#include "render_object/SplineRenderer.h"


#define GL_NORMALS                        0x0011

void MeshRenderer::setMesh(const Mesh *m) {
    mesh = m;
    arrayBuf.bind();
    arrayBuf.allocate(mesh->vertices.data(), mesh->vertices.count() * sizeof(VertexData));

    indexBuf.bind();
    indexBuf.allocate(mesh->indices.data(), mesh->indices.count() * sizeof(GLuint));

}

void MeshRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/mesh/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/mesh/fshader.glsl");

        program->link();
    }
}

void MeshRenderer::paint() {
    program->bind();
    arrayBuf.bind();
    indexBuf.bind();
    int projection = program->uniformLocation("projection");
    if (projection != -1) {
        program->setUniformValue(projection, sceneData->camera.getProjection());
    }

    int u_ModelView = program->uniformLocation("modelview");
    if (u_ModelView != -1) {
        program->setUniformValue(u_ModelView, sceneData->camera.getView() * model);
    }

    int u_NormalMat = program->uniformLocation("normalMat");
    if (u_NormalMat != -1)
    {
        auto normalMat = (sceneData->camera.getView()* model).inverted().transposed();
        //QMatrix4x4 normalMat;
        //normalMat.setToIdentity();
        program->setUniformValue(u_NormalMat, normalMat);
    }

    int LightPosition = program->uniformLocation("lightPos");
    if (LightPosition != -1)
    {
        program->setUniformValue(LightPosition, 10, 10, 10);
    }

    int Kd = program->uniformLocation("Kd");
    if (Kd != -1)
    {
        program->setUniformValue(Kd, 0.5f);
    }

    int Ka = program->uniformLocation("Ka");
    if (Ka != -1)
    {
        program->setUniformValue(Ka, 0.8f);
    }

    int Ks = program->uniformLocation("Ks");
    if (Ks != -1)
    {
        program->setUniformValue(Ks, 0.3f);
    }

    int Shininess = program->uniformLocation("shininessVal");
    if (Shininess != -1)
    {
        program->setUniformValue(Shininess, 20.f);
    }
    int ambientColor = program->uniformLocation("ambientColor");
    if (ambientColor != -1)
    {
        program->setUniformValue(ambientColor, 0.133, 0.38, 0.125);
    }

    int diffuseColor = program->uniformLocation("diffuseColor");
    if (diffuseColor != -1)
    {
        program->setUniformValue(diffuseColor, 0.8, 0.4, 0);
    }

    int specularColor = program->uniformLocation("specularColor");
    if (specularColor != -1)
    {
        program->setUniformValue(specularColor, 1, 1, 1);
    }

    int vertexLocation = program->attributeLocation("a_Position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));

    quintptr offset = 0;

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    if (drawMode == GL_POINTS) {
        glPolygonMode(GL_FRONT, GL_POINT);
        glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_INT, nullptr);
        glPolygonMode(GL_FRONT, GL_FILL);
    }

    if (drawMode == GL_LINES) {
        glPolygonMode(GL_FRONT, GL_LINE);
        glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_INT, nullptr);
        glPolygonMode(GL_FRONT, GL_FILL);
    }

    if (drawMode == GL_TRIANGLES) {

        int normalLocation = program->attributeLocation("a_Norm");
        program->enableAttributeArray(normalLocation);
        program->setAttributeBuffer(normalLocation, GL_FLOAT, offset, 3, sizeof(VertexData));


        glPolygonMode(GL_FRONT, GL_FILL);
        glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_INT, nullptr);
    }

    if (drawMode == GL_NORMALS) {
        glDrawElements(GL_LINES, mesh->indices.count(), GL_UNSIGNED_INT, nullptr);
    }

    arrayBuf.release();
    indexBuf.release();
    program->release();
}

void MeshRenderer::setDrawMode(GLenum mode) {
    drawMode = mode;
}

