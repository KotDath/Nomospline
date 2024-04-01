#pragma once

#include <QtQmlIntegration>
#include <QtConcurrent>

#include "engine/MeshLoader.h"
#include "engine/SplineLoader.h"
#include "scene.h"

QVector<QVector4D> filterPoints(const QVector<QVector4D>& points);
void asyncIntersection();
void asyncEvaluate();

class WindowUtils : public QObject {
Q_OBJECT
    Q_PROPERTY(QQuickWindow* window READ getWindow WRITE setWindow NOTIFY windowChanged)
    Q_PROPERTY(Scene* scene READ getScene WRITE setScene NOTIFY sceneChanged)
    QML_ELEMENT
public:
    WindowUtils();
    QQuickWindow* getWindow() { return window;}
    void setWindow(QQuickWindow* win) {window = win;}

    Scene* getScene() { return scene;}
    void setScene(Scene* sce) {scene = sce;}

    signals:
    void windowChanged(QQuickWindow* window);
    void sceneChanged(Scene* scene);
public:
    Q_INVOKABLE QUrl getHomeDirectory();

    Q_INVOKABLE void clear();

    Q_INVOKABLE void importMesh(const QString &path);

    Q_INVOKABLE void evaluate();

    Q_INVOKABLE void calculateIntersection();

    Q_INVOKABLE void drawMeshes(bool checked);

    Q_INVOKABLE void drawEvaluatedMeshes(bool checked);

    Q_INVOKABLE void drawSplines(bool checked);

    Q_INVOKABLE void setForward();

    Q_INVOKABLE void setBackward();

    Q_INVOKABLE void setRight();

    Q_INVOKABLE void setLeft();

    Q_INVOKABLE void setUp();

    Q_INVOKABLE void setDown();

private:
    SceneData* data = nullptr;
    QFutureWatcher<void> watcherIntersection;
    QFutureWatcher<void> watcherEvalutation;

    MeshLoader meshLoader;
    SplineLoader splineLoader;

    Scene *scene = nullptr;
    QQuickWindow *window = nullptr;
};
