#include <QMouseEvent>

#include "widget/OpenGLWindow.h"

#define GL_NORMALS                        0x0011

void OpenGLWindow::initShaders()
{

}

void OpenGLWindow::resizeGL(int w, int h)
{
    qreal aspect = qreal(w) / qreal(h ? h : 1);
    const qreal zNear = 0.1, zFar = 10000.0, fov = 60.0;

    // Set perspective projection
    camera.setPerpective(fov, aspect, zNear, zFar);

    QOpenGLWidget::resizeGL(w, h);
}

void OpenGLWindow::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);


    meshProgram.bind();

    // Set modelview-projection model
    meshProgram.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);
    if (isTriangles) {
        for (const auto &mesh: meshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }
    }

    linesProgram.bind();

    // Set modelview-projection model
    linesProgram.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);

    if (isLines) {
        for (const auto &mesh: meshes)
        {
            engine->draw(&linesProgram, mesh, GL_LINES);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&linesProgram, mesh, GL_LINES);
        }
    }

    if (isNormal) {
        for (const auto &mesh: normalMeshes)
        {
            engine->draw(&linesProgram, mesh, GL_NORMALS);
        }
    }

    pointsProgram.bind();

    // Set modelview-projection model
    pointsProgram.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);
    if (isPoint) {
        for (const auto &mesh: meshes)
        {
            engine->draw(&pointsProgram, mesh, GL_POINTS);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&pointsProgram, mesh, GL_POINTS);
        }

    }




    QOpenGLWidget::paintGL();
}

void OpenGLWindow::initializeGL()
{

    initializeOpenGLFunctions();

    camera.setPosition(0, 5, 10);
    camera.setViewPoint(0, 0, 0);
    camera.setUpVector(0, 1, 0);
    // Compile vertex shader
    if (!meshProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/mesh/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!meshProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/mesh/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!meshProgram.link())
        close();

    // Bind shader pipeline for use
    if (!meshProgram.bind())
        close();

    if (!pointsProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/points/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!pointsProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/points/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!pointsProgram.link())
        close();

    // Bind shader pipeline for use
    if (!pointsProgram.bind())
        close();

    if (!linesProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lines/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!linesProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lines/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!linesProgram.link())
        close();

    // Bind shader pipeline for use
    if (!linesProgram.bind())
        close();

    engine = new GeometryEngine;
    glClearColor(0, 0, 0, 1);
    initShaders();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_PROGRAM_POINT_SIZE);


    QOpenGLWidget::initializeGL();
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent *event)
{

    camera.rotateCameraAroundY(previousMousePosition.x() - event->pos().x());
    camera.rotateCameraAroundX(previousMousePosition.y() - event->pos().y());
    qDebug() << previousMousePosition.y() - event->pos().y();
    previousMousePosition = event->pos();
    update();

    QWidget::mouseMoveEvent(event);
}

void OpenGLWindow::mousePressEvent(QMouseEvent *event)
{
    previousMousePosition = event->pos();

    QWidget::mousePressEvent(event);
}

void OpenGLWindow::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0)
    {
        camera.zoomIn(2);
        update();
    }

    if (event->angleDelta().y() < 0)
    {
        camera.zoomOut(2);
        update();
    }

    QWidget::wheelEvent(event);
}

void OpenGLWindow::addGeometry(Mesh *newMesh)
{
    meshes.append(newMesh);
}

OpenGLWindow::~OpenGLWindow()
{
    clear();
    delete engine;


}

void OpenGLWindow::clear()
{
    for (auto mesh : meshes) {
        delete mesh;
    }

    for (auto mesh : splineMeshes) {
        delete mesh;
    }

    for (auto mesh : normalMeshes) {
        delete mesh;
    }

    for (auto spline : splines) {
        delete spline;
    }

    meshes.clear();
    splineMeshes.clear();
    normalMeshes.clear();
    splines.clear();
    update();
}

void OpenGLWindow::addSpline(NURBS *newSpline)
{
    splines.append(newSpline);
}

void OpenGLWindow::evaluateSplines()
{
    for (auto mesh : splineMeshes) {
        delete mesh;
    }
    splineMeshes.clear();

    for (const auto& spline : splines) {
        splineMeshes.append(spline->evaluate());
    }

    update();
}

void OpenGLWindow::setPoint(bool point)
{
    isPoint = point;
    update();
}

void OpenGLWindow::setLine(bool line)
{
    isLines = line;
    update();
}

void OpenGLWindow::setTriangle(bool triangle)
{
    isTriangles = triangle;
    update();
}

void OpenGLWindow::setNormal(bool normal)
{
    isNormal = normal;
    if (isNormal) {
        /*for (auto mesh : meshes) {
            auto* newMesh = new Mesh();
            for (const auto& vertice : mesh->vertices) {
                auto tmp = vertice;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);
                tmp.position += vertice.normal;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);

            }

            normalMeshes.append(newMesh);
        }*/

        for (auto mesh : splineMeshes) {
            auto* newMesh = new Mesh();
            for (const auto& vertice : mesh->vertices) {
                auto tmp = vertice;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);
                tmp.position += vertice.normal * normalSize;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);
            }

            normalMeshes.append(newMesh);
        }
    } else {
        for (auto mesh : normalMeshes) {
            delete mesh;
        }

        normalMeshes.clear();
    }
    update();
}

void OpenGLWindow::setNormalLength(float size)
{
    normalSize = size;
}
