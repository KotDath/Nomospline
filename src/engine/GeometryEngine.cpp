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

void GeometryEngine::draw(QOpenGLShaderProgram *program, const Mesh *mesh, GLenum drawMode)
{
    if (mesh != nullptr)
    {

        arrayBuf.bind();
        arrayBuf.allocate(mesh->vertices.data(), mesh->vertices.count() * sizeof(VertexData));

        indexBuf.bind();
        indexBuf.allocate(mesh->indices.data(), mesh->indices.count() * sizeof(GLushort));

        quintptr offset = 0;

        int vertexLocation = program->attributeLocation("a_position");
        program->enableAttributeArray(vertexLocation);
        program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));


        // Offset for texture coordinate
        offset += sizeof(QVector3D);

        if (drawMode == GL_POINTS) {
            glDrawArrays(drawMode, 0, mesh->vertices.count());
        } else {
            glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_SHORT, nullptr);
        }

    }

}

void GeometryEngine::init()
{

}
