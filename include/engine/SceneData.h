#pragma once

#include "Camera.h"
#include "structures/Mesh.h"
#include "structures/NURBS.h"

class SceneData {
private:
    static SceneData *instance;

    SceneData() {}

    SceneData(const SceneData &);

    SceneData &operator=(SceneData &);

public:
    static SceneData *getInstance() {
        if (!instance)
            instance = new SceneData();
        return instance;
    }

    Camera camera;
    QVector<Mesh*> meshes;
    QVector<NURBS*> splines;
    QVector<Mesh*> splineMeshes;
    Mesh* intersectionPoints;

    bool isRenderMeshes = false;
    bool isRenderEvaluate = false;
    bool isRenderSpline = false;

    bool intersectionBlocker = false;
    bool evalutaionBlocker = false;
};