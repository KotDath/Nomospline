#include <QWidget>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>


#include "widget/MainWindow.h"
#include "widget/OpenGLWindow.h"
#include "engine/MeshLoader.h"

MainWindow::MainWindow() : meshLoader(new MeshLoader), openglWidget(new OpenGLWindow)
{
    auto *fileMenu = menuBar()->addMenu(tr("&File"));
    auto *importAction = new QAction(tr("&Import"), this);
    fileMenu->addAction(importAction);
    connect(importAction, &QAction::triggered, this, &MainWindow::importFile);

    auto *mainWidget = new QWidget;
    setCentralWidget(mainWidget);



    auto *layout = new QVBoxLayout;

    layout->addWidget(openglWidget);

    mainWidget->setLayout(layout);


}

void MainWindow::importFile()
{
    auto fileName = QFileDialog::getOpenFileName(this,
                                            QDir::homePath(), "", tr("NURBS Spline or mesh (*.obj *.d3m)"));
    if (fileName.endsWith(".obj", Qt::CaseInsensitive)) {
        qDebug() << "mesh";
        auto* mesh = meshLoader->load(fileName);
        openglWidget->setGeometry(mesh);
    }

    if (fileName.endsWith(".d3m", Qt::CaseInsensitive)) {
        qDebug() << "NURBS";
    }

    qDebug() << fileName;
}
