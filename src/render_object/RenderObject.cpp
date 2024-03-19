#include "render_object/RenderObject.h"
#include "engine/SceneData.h"

RenderObject::RenderObject() : indexBuf(QOpenGLBuffer::IndexBuffer) {
    sceneData = SceneData::getInstance();
    initializeOpenGLFunctions();
    arrayBuf.create();
    indexBuf.create();
}

RenderObject::~RenderObject() {
    arrayBuf.destroy();
    indexBuf.destroy();
}
