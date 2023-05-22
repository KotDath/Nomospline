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
    void importFile();

private:
    MeshLoader meshLoader;
    SplineLoader splineLoader;
    OpenGLWindow *openglWidget{};
};