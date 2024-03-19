#pragma once

#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include "engine/Camera.h"
#include "engine/SceneData.h"

class RenderObject : protected QOpenGLFunctions {
public:
    RenderObject();
    ~RenderObject();
    virtual void initShaders() = 0;
    virtual void paint() = 0;

protected:
    SceneData* sceneData;
    QMatrix4x4 model;
    QOpenGLShaderProgram* program = nullptr;
    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
};
