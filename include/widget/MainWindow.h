#pragma once

#include <QMainWindow>

#include "engine/FileLoader.h"
#include "OpenGLWindow.h"

class MainWindow : public QMainWindow
{

public:
    MainWindow();

private slots:
    void importFile();

private:
    FileLoader* meshLoader;
    OpenGLWindow *openglWidget{};
};