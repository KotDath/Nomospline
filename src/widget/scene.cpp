// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#define GL_NORMALS                        0x0011

#include "widget/scene.h"

// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause


#include <QtQuick/qquickwindow.h>
#include <QOpenGLShaderProgram>
#include <QOpenGLContext>
#include <QtCore/QRunnable>

Scene::Scene()
        : m_renderer(nullptr) {
    setAcceptedMouseButtons(Qt::AllButtons);
    connect(this, &QQuickItem::windowChanged, this, &Scene::handleWindowChanged);
}

void Scene::handleWindowChanged(QQuickWindow *win) {
    if (win) {
        qDebug() << "Connect";
        connect(win, &QQuickWindow::beforeSynchronizing, this, &Scene::sync, Qt::DirectConnection);
        connect(win, &QQuickWindow::sceneGraphInvalidated, this, &Scene::cleanup, Qt::DirectConnection);
    }
}

void Scene::cleanup() {
    //delete m_renderer;
    //m_renderer = nullptr;
}

class CleanupJob : public QRunnable {
public:
    CleanupJob(SceneRenderer *renderer) : m_renderer(renderer) {}

    void run() override { /*delete m_renderer;*/ }

private:
    SceneRenderer *m_renderer;
};

void Scene::releaseResources() {
    window()->scheduleRenderJob(new CleanupJob(m_renderer), QQuickWindow::BeforeSynchronizingStage);
    m_renderer = nullptr;
}

SceneRenderer::~SceneRenderer() {
    delete m_program;
}

void Scene::sync() {
    if (!m_renderer) {
        m_renderer = new SceneRenderer();
        connect(window(), &QQuickWindow::beforeRendering, m_renderer, &SceneRenderer::init, Qt::DirectConnection);
        connect(window(), &QQuickWindow::beforeRenderPassRecording, m_renderer, &SceneRenderer::paint,
                Qt::DirectConnection);
        connect(this, &QQuickItem::widthChanged, m_renderer, [this]() {
            QSize s = size().toSize();
            m_renderer->setViewportSize(s * window()->devicePixelRatio());
            m_renderer->resize();
        }, Qt::DirectConnection);
        connect(this, &QQuickItem::heightChanged, m_renderer, [this]() {
            QSize s = size().toSize();
            m_renderer->setViewportSize(s * window()->devicePixelRatio());
            m_renderer->resize();
        }, Qt::DirectConnection);
    }
    QSize s = size().toSize();
    m_renderer->setViewportSize(s * window()->devicePixelRatio());
    m_renderer->setPosition(QSize(x(), y()));
    m_renderer->setWindow(window());
}

void Scene::mouseMoveEvent(QMouseEvent *event) {
    m_renderer->mouseMoveEvent(event);
}

void Scene::mousePressEvent(QMouseEvent *event) {
    m_renderer->mousePressEvent(event);
}

void Scene::wheelEvent(QWheelEvent *event) {
    m_renderer->wheelEvent(event);
}

void Scene::addGeometry(Mesh *mesh) {
    SceneData::getInstance()->meshes.push_back(mesh);
}

void Scene::addSpline(NURBS *spl) {
    SceneData::getInstance()->splines.push_back(spl);
}

void SceneRenderer::init() {
    if (!m_program) {
        QSGRendererInterface *rif = m_window->rendererInterface();
        Q_ASSERT(rif->graphicsApi() == QSGRendererInterface::OpenGL);

        initializeOpenGLFunctions();

        initShaders();
        resize();

    }
}

void SceneRenderer::paint() {
    m_window->beginExternalCommands();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glDisable(GL_BLEND);



    glViewport(m_position.width(), m_position.height(), m_viewportSize.width(), m_viewportSize.height());
    glDisable(GL_BLEND);
    backgroundRenderer.paint();
    if (sceneData->isRenderMeshes) {
        for (const auto &mesh: sceneData->meshes) {
            meshRenderer.setMesh(mesh);
            meshRenderer.paint();
            normalRenderer.setMesh(mesh);
            normalRenderer.paint();
        }

        if (!sceneData->intersectionBlocker && sceneData->intersectionPoints) {
            pointRenderer.setMesh(sceneData->intersectionPoints);
            pointRenderer.paint();
        }
    }

    if (sceneData->isRenderEvaluate && !sceneData->evalutaionBlocker) {
        for (const auto &mesh: sceneData->splineMeshes) {
            meshRenderer.setMesh(mesh);
            meshRenderer.paint();
        }
    }

    if (sceneData->isRenderSpline) {
        for (const auto &spl: sceneData->splines) {
            splineRenderer.setSpline(spl);
            splineRenderer.paint();
        }
    }


    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glViewport(m_position.width() + m_viewportSize.width() - 150, 0,
               150, 150);

    gizmosRenderer.paint();

    m_window->endExternalCommands();
    sceneData->camera.interp(0.9, m_window);
}

void SceneRenderer::resize() {
    qreal aspect = qreal(m_viewportSize.width()) / qreal(m_viewportSize.height() ? m_viewportSize.height() : 1);
    const qreal zNear = 0.1, zFar = 10000.0, fov = 60.0;

    // Set perspective projection
    sceneData->camera.setPerpective(fov, aspect, zNear, zFar);

}

void SceneRenderer::initShaders() {
    backgroundRenderer.initShaders();
    meshRenderer.initShaders();
    gizmosRenderer.initShaders();
    splineRenderer.initShaders();
    pointRenderer.initShaders();
    normalRenderer.initShaders();
}

void SceneRenderer::mouseMoveEvent(QMouseEvent *event) {
    sceneData->camera.rotateCameraAroundY(previousMousePosition.x() - event->pos().x());
    sceneData->camera.rotateCameraAroundX(previousMousePosition.y() - event->pos().y());
    previousMousePosition = event->pos();
    m_window->update();

}

void SceneRenderer::mousePressEvent(QMouseEvent *event) {
    previousMousePosition = event->pos();
    m_window->update();

}

void SceneRenderer::wheelEvent(QWheelEvent *event) {
    if (event->angleDelta().y() > 0) {

        sceneData->camera.zoomIn(2);
        m_window->update();

    }

    if (event->angleDelta().y() < 0) {
        sceneData->camera.zoomOut(2);
        m_window->update();
    }

}

SceneRenderer::SceneRenderer() {
    sceneData = SceneData::getInstance();
    /*
    mesh.vertices = {VertexData(QVector3D(1, -1, -1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(1, -1, 1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(-1, -1, 1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(-1, -1, -1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(1, 1, -1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(1, 1, 1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(-1, 1, 1), QVector3D(0, 0, 0)),
                     VertexData(QVector3D(-1, 1, -1), QVector3D(0, 0, 0)),
    };

    mesh.indices = {
            0, 1, 2,
            2, 3, 0,
            // right
            1, 5, 6,
            6, 2, 1,
            // back
            7, 6, 5,
            5, 4, 7,
            // left
            4, 0, 3,
            3, 7, 4,
            // bottom
            4, 5, 1,
            1, 0, 4,
            // top
            3, 2, 6,
            6, 7, 3

    };*/

    sceneData->camera.setPosition(5, 5, 5);
    sceneData->camera.setViewPoint(0, 0, 0);
    sceneData->camera.setUpVector(0, 1, 0);

}
