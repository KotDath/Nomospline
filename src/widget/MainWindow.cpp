#include <QWidget>
#include <QVBoxLayout>
#include <QMenuBar>
#include <QFileDialog>


#include "widget/MainWindow.h"
#include "widget/OpenGLWindow.h"
#include "engine/MeshLoader.h"

MainWindow::MainWindow() : meshLoader(), splineLoader(), openglWidget(new OpenGLWindow)
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
    if (fileName.endsWith(".obj", Qt::CaseInsensitive))
    {
        bool isSpline = false;
        QFile file(fileName);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream stream(&file);
            while (!stream.atEnd())
            {
                QString line = stream.readLine();
                if (line.contains("cstype rat bspline", Qt::CaseInsensitive))
                {
                    isSpline = true;
                    break;
                }
            }
            file.close();
        }


        if (isSpline)
        {
            auto *spline = splineLoader.load(fileName);

        } else
        {
            auto *mesh = meshLoader.load(fileName);
            openglWidget->addGeometry(mesh);
        }


    }

    qDebug() << fileName;
}
