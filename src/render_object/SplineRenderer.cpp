#include "render_object/SplineRenderer.h"
#include "render_object/PointRenderer.h"


void SplineRenderer::setSpline(const NURBS *spline) {
    spl = spline;
    arrayBuf.bind();
    QVector<QVector3D> points;
    QVector<GLushort> indices;
    GLushort i = 0;
    auto len = spl->controlPoints[0].count();
    for (const auto &row: spl->controlPoints) {
        for (const auto &elem: row) {
            points.append(QVector3D(elem.x(), elem.y(), elem.z()));

            if ((i + 1) % len != 0 && i / len < spl->controlPoints.count() - 1) {
                indices.append(i);
                indices.append(i + 1);

                indices.append(i + len);
                indices.append(i + len + 1);

                indices.append(i);
                indices.append(i + len);

                indices.append(i + 1);
                indices.append(i + len + 1);
            }

            ++i;
        }
    }


    arrayBuf.allocate(points.data(), points.count() * sizeof(QVector3D));
    indexBuf.bind();
    indexBuf.allocate(indices.data(), indices.count() * sizeof(GLushort));
    count = points.count();
    spline_count = indices.count();

}

void SplineRenderer::initShaders() {
    if (!program) {
        program = new QOpenGLShaderProgram();
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/points/vshader.glsl");
        program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/points/fshader.glsl");

        program->link();
    }

    if (!grid_program) {
        grid_program = new QOpenGLShaderProgram();
        grid_program->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lines/vshader.glsl");
        grid_program->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment,
                                                       ":/shaders/lines/fshader.glsl");

        grid_program->link();
    }
}

void SplineRenderer::paint() {
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
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);

    glDrawArrays(GL_POINTS, 0, count);

    program->release();


    grid_program->bind();
    mvp_matrix = program->uniformLocation("mvp_matrix");
    if (mvp_matrix != -1) {

        grid_program->setUniformValue(mvp_matrix,
                                 sceneData->camera.getProjection() * sceneData->camera.getView() * model);
    }

    vertexLocation = grid_program->attributeLocation("a_Position");
    grid_program->enableAttributeArray(vertexLocation);
    grid_program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(GLfloat) * 3);
    glDrawElements(GL_TRIANGLES, spline_count, GL_UNSIGNED_SHORT, nullptr);

    arrayBuf.release();
    indexBuf.release();
    grid_program->release();

}

