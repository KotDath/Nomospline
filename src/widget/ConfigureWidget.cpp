#include "widget/ConfigureWidget.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>

ConfigureWidget::ConfigureWidget(OpenGLWindow *parent)
{
    auto *mainLayout = new QVBoxLayout(this);
    auto *checkboxPoint = new QCheckBox(tr("Points"), this);
    auto *checkboxLine = new QCheckBox(tr("Lines"), this);
    auto *checkboxTriangles = new QCheckBox(tr("Triangles"), this);
    auto *checkboxNormals = new QCheckBox(tr("Normals"), this);

    mainLayout->addWidget(checkboxPoint);
    mainLayout->addWidget(checkboxLine);
    mainLayout->addWidget(checkboxTriangles);
    mainLayout->addWidget(checkboxNormals);

    connect(checkboxPoint, &QCheckBox::stateChanged, parent, &OpenGLWindow::setPoint);
    connect(checkboxLine, &QCheckBox::stateChanged, parent, &OpenGLWindow::setLine);
    connect(checkboxTriangles, &QCheckBox::stateChanged, parent, &OpenGLWindow::setTriangle);
    connect(checkboxNormals, &QCheckBox::stateChanged, parent, &OpenGLWindow::setNormal);
    setLayout(mainLayout);
}
