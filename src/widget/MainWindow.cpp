#include <QWidget>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>


#include "widget/MainWindow.h"
#include "widget/OpenGLWindow.h"
#include "engine/MeshLoader.h"

MainWindow::MainWindow() : meshLoader(new MeshLoader), openglWidget(new OpenGLWindow)
{

    setWindowIcon(QIcon(":/images/img.png"));
    QFont fileFont;
    fileFont.setPointSize(16);
    menuBar()->setFont(fileFont);

    auto *fileMenu = menuBar()->addMenu(tr("&File"));

    auto *importAction = new QAction(tr("&Import"), this);
    fileMenu->addAction(importAction);
    connect(importAction, &QAction::triggered, this, &MainWindow::importFile);

    auto *clearAction = new QAction(tr("&Clear Scene"), this);
    fileMenu->addAction(clearAction);
    connect(clearAction, &QAction::triggered, openglWidget, &OpenGLWindow::clear);

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
        openglWidget->addGeometry(mesh);
    }

    if (fileName.endsWith(".d3m", Qt::CaseInsensitive)) {
        qDebug() << "NURBS";
    }

    qDebug() << fileName;
}
