#pragma once

#include <QMainWindow>

#include "engine/MeshLoader.h"
#include "engine/SplineLoader.h"

#include "OpenGLWindow.h"

class MainWindow : public QMainWindow
{

public:
    MainWindow();

private slots:
    void importMesh();

private:
    MeshLoader meshLoader;
    SplineLoader splineLoader;
    OpenGLWindow *openglWidget{};
};