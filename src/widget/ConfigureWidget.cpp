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

    mainLayout->addWidget(checkboxPoint);
    mainLayout->addWidget(checkboxLine);
    mainLayout->addWidget(checkboxTriangles);

    connect(checkboxPoint, &QCheckBox::stateChanged, parent, &OpenGLWindow::setPoint);
    connect(checkboxLine, &QCheckBox::stateChanged, parent, &OpenGLWindow::setLine);
    connect(checkboxTriangles, &QCheckBox::stateChanged, parent, &OpenGLWindow::setTriangle);
    setLayout(mainLayout);
}
