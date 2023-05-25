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

        if (drawMode == GL_POINTS)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_SHORT, nullptr);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        if (drawMode == GL_LINES)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_SHORT, nullptr);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        if (drawMode == GL_TRIANGLES)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(drawMode, mesh->indices.count(), GL_UNSIGNED_SHORT, nullptr);
        }

    }

}

void GeometryEngine::init()
{

}