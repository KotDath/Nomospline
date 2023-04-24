#include "engine/GeometryEngine.h"

GeometryEngine::GeometryEngine()
        : indexBuf(QOpenGLBuffer::IndexBuffer)
{

    initializeOpenGLFunctions();

    arrayBuf.create();
    indexBuf.create();


}

GeometryEngine::~GeometryEngine()
{
    arrayBuf.destroy();
    indexBuf.destroy();
}

void GeometryEngine::draw(QOpenGLShaderProgram *program, const Mesh *mesh)
{
    if (mesh != nullptr)
    {

        arrayBuf.bind();
        arrayBuf.allocate(mesh->vertices, mesh->verticesCount * sizeof(VertexData));

        indexBuf.bind();
        indexBuf.allocate(mesh->indices, mesh->indicesCount * sizeof(GLushort));

        quintptr offset = 0;

        int vertexLocation = program->attributeLocation("a_position");
        program->enableAttributeArray(vertexLocation);
        program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));


        // Offset for texture coordinate
        offset += sizeof(QVector3D);

        glDrawElements(GL_TRIANGLES, mesh->indicesCount, GL_UNSIGNED_SHORT, nullptr);
    }

}

void GeometryEngine::init()
{

}
