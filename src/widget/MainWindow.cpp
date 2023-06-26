#include <QWidget>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QSplitter>

#include "widget/MainWindow.h"
#include "widget/OpenGLWindow.h"
#include "widget/ConfigureWidget.h"

MainWindow::MainWindow() : meshLoader(), splineLoader(), openglWidget(new OpenGLWindow)
{

    setWindowIcon(QIcon(":/images/img.png"));
    QFont fileFont;
    fileFont.setPointSize(16);
    menuBar()->setFont(fileFont);

    auto *fileMenu = menuBar()->addMenu(tr("&File"));

    auto *importAction = new QAction(tr("&Import"), this);
    fileMenu->addAction(importAction);
    connect(importAction, &QAction::triggered, this, &MainWindow::importMesh);

    auto *clearAction = new QAction(tr("&Clear Scene"), this);
    fileMenu->addAction(clearAction);
    connect(clearAction, &QAction::triggered, openglWidget, &OpenGLWindow::clear);

    auto *evaluateAction = new QAction(tr("&Evaluate"), this);
    fileMenu->addAction(evaluateAction);
    connect(evaluateAction, &QAction::triggered, openglWidget, &OpenGLWindow::evaluateSplines);

    auto *calculatePoints = new QAction(tr("&Calculate Intersection"), this);
    fileMenu->addAction(calculatePoints);
    connect(calculatePoints, &QAction::triggered, openglWidget, &OpenGLWindow::calculateIntersection);

    auto *mainWidget = new QSplitter(Qt::Horizontal, this);;
    setCentralWidget(mainWidget);


    auto *layout = new QHBoxLayout;


    layout->addWidget(openglWidget);

    auto *configureWidget = new ConfigureWidget(openglWidget);

    layout->addWidget(configureWidget);

    mainWidget->setLayout(layout);


}

void MainWindow::importMesh()
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
            openglWidget->addSpline(spline);
        } else
        {
            auto *mesh = meshLoader.load(fileName);
            openglWidget->addGeometry(mesh);
        }


    }

    qDebug() << fileName;
}
