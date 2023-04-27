#include <QMouseEvent>

#include "widget/OpenGLWindow.h"

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


    program.bind();

    // Set modelview-projection model
    program.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);
    for (const auto &mesh: meshes)
    {
        engine->draw(&program, mesh);
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
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();

    engine = new GeometryEngine;
    glClearColor(0, 1, 0, 1);
    initShaders();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);


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
    for (int i = 0; i < meshes.count(); ++i)
    {
        delete meshes[i];
    }

    meshes.clear();
    update();
}
