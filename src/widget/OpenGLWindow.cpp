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


    meshProgram.link();
    meshProgram.bind();

    // Set modelview-projection model
    int projection = meshProgram.uniformLocation("projection");
    if (projection != -1)
    {
        meshProgram.setUniformValue(projection, camera.getProjection());
    }

    int u_ModelView = meshProgram.uniformLocation("modelview");
    if (u_ModelView != -1)
    {
        meshProgram.setUniformValue(u_ModelView, camera.getView() * model);
    }

    int u_NormalMat = meshProgram.uniformLocation("normalMat");
    if (u_NormalMat != -1)
    {
        auto normalMat = (camera.getView() * model).inverted().transposed();
        //QMatrix4x4 normalMat;
        //normalMat.setToIdentity();
        meshProgram.setUniformValue(u_NormalMat, normalMat);
    }

    int LightPosition = meshProgram.uniformLocation("lightPos");
    if (LightPosition != -1)
    {
        meshProgram.setUniformValue(LightPosition, 10, 10, 10);
    }

    int Kd = meshProgram.uniformLocation("Kd");
    if (Kd != -1)
    {
        meshProgram.setUniformValue(Kd, 0.5f);
    }

    int Ka = meshProgram.uniformLocation("Ka");
    if (Ka != -1)
    {
        meshProgram.setUniformValue(Ka, 0.8f);
    }

    int Ks = meshProgram.uniformLocation("Ks");
    if (Ks != -1)
    {
        meshProgram.setUniformValue(Ks, 0.3f);
    }

    int Shininess = meshProgram.uniformLocation("shininessVal");
    if (Shininess != -1)
    {
        meshProgram.setUniformValue(Shininess, 20.f);
    }
    int ambientColor = meshProgram.uniformLocation("ambientColor");
    if (ambientColor != -1)
    {
        meshProgram.setUniformValue(ambientColor, 0.133, 0.38, 0.125);
    }

    int diffuseColor = meshProgram.uniformLocation("diffuseColor");
    if (diffuseColor != -1)
    {
        meshProgram.setUniformValue(diffuseColor, 0.8, 0.4, 0);
    }

    int specularColor = meshProgram.uniformLocation("specularColor");
    if (specularColor != -1)
    {
        meshProgram.setUniformValue(specularColor, 1, 1, 1);
    }


    if (isTriangles)
    {
        for (const auto &mesh: meshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }
    }
    /*
    if (isTriangles) {
        for (const auto &mesh: meshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&meshProgram, mesh, GL_TRIANGLES);
        }
    }*/

    linesProgram.bind();

    // Set modelview-projection model
    linesProgram.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);

    if (isLines)
    {
        for (const auto &mesh: meshes)
        {
            engine->draw(&linesProgram, mesh, GL_LINES);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&linesProgram, mesh, GL_LINES);
        }
    }

    if (isNormal)
    {
        for (const auto &mesh: normalMeshes)
        {
            engine->draw(&linesProgram, mesh, GL_NORMALS);
        }
    }

    pointsProgram.bind();

    // Set modelview-projection model
    pointsProgram.setUniformValue("mvp_matrix", camera.getProjection() * camera.getView() * model);
    if (isPoint)
    {
        /*for (const auto &mesh: meshes)
        {
            engine->draw(&pointsProgram, mesh, GL_POINTS);
        }

        for (const auto &mesh: splineMeshes)
        {
            engine->draw(&pointsProgram, mesh, GL_POINTS);
        }*/

        engine->draw(&pointsProgram, intersectionPoints, GL_POINTS);

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
    glClearColor(1, 1, 1, 1);
    initShaders();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);


    QOpenGLWidget::initializeGL();
}

void OpenGLWindow::mouseMoveEvent(QMouseEvent *event)
{

    camera.rotateCameraAroundY(previousMousePosition.x() - event->pos().x());
    camera.rotateCameraAroundX(previousMousePosition.y() - event->pos().y());
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
    for (auto mesh: meshes)
    {
        delete mesh;
    }

    for (auto mesh: splineMeshes)
    {
        delete mesh;
    }

    for (auto mesh: normalMeshes)
    {
        delete mesh;
    }

    for (auto spline: splines)
    {
        delete spline;
    }

    meshes.clear();
    splineMeshes.clear();
    normalMeshes.clear();
    splines.clear();
    delete intersectionPoints;

    update();
}

void OpenGLWindow::addSpline(NURBS *newSpline)
{
    splines.append(newSpline);
}

void OpenGLWindow::evaluateSplines()
{
    for (auto mesh: splineMeshes)
    {
        delete mesh;
    }
    splineMeshes.clear();

    for (const auto &spline: splines)
    {
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
    if (isNormal)
    {
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

        for (auto mesh: splineMeshes)
        {
            auto *newMesh = new Mesh();
            for (const auto &vertice: mesh->vertices)
            {
                auto tmp = vertice;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);
                tmp.position += vertice.normal * normalSize;
                newMesh->vertices.append(tmp);
                newMesh->indices.append(newMesh->vertices.count() - 1);
            }

            normalMeshes.append(newMesh);
        }
    } else
    {
        for (auto mesh: normalMeshes)
        {
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

void OpenGLWindow::calculateIntersection()
{
    intersectionPoints = new Mesh();
    for (int i = 0; i < splines.length(); ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            auto intersection = splines[j]->getInitialPoints(splines[i]);
            for (auto k : intersection)
            {
                qDebug() << "Difference: " << (splines[i]->getPoint(k.z(), k.w()) - splines[j]->getPoint(k.x(), k.y())).length();
                intersectionPoints->vertices.append(splines[i]->getPoint(k.z(), k.w()));
                intersectionPoints->indices.append(intersectionPoints->indices.length());
            }

           for (auto point : intersection) {
                auto iterIntersection = splines[j]->iterPoints(splines[i], point);
                for (auto p : iterIntersection) {
                    intersectionPoints->vertices.append(p);
                    intersectionPoints->indices.append(intersectionPoints->indices.length());
                }
            }
        }
    }

}
