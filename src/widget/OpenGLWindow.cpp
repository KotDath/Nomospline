#include <QTimer>
#include "widget/OpenGLWindow.h"

void OpenGLWindow::initShaders()
{

}

void OpenGLWindow::resizeGL(int w, int h)
{
    qreal aspect = qreal(w) / qreal(h ? h : 1);
    const qreal zNear = 0.1, zFar = 100.0, fov = 60.0;

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

    model.translate(0.0, 0.0, 0.0);
    model.rotate(0.3, 0, 1, 0);

    camera.setPosition(0, 1, 5);
    camera.setViewPoint(0, 0, 0);
    camera.setUpVector(0, 1, 0);

    // Set modelview-projection model
    program.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);
    engine->draw(&program, mesh);

    QOpenGLWidget::paintGL();
}

void OpenGLWindow::initializeGL()
{
    mesh = nullptr;

    initializeOpenGLFunctions();
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(8);
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