#include "widget/ConfigureWidget.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleValidator>

ConfigureWidget::ConfigureWidget(OpenGLWindow *parent)
{
    opengl = parent;
    auto *mainLayout = new QVBoxLayout(this);
    auto *checkboxPoint = new QCheckBox(tr("Points"), this);
    auto *checkboxLine = new QCheckBox(tr("Lines"), this);
    auto *checkboxTriangles = new QCheckBox(tr("Triangles"), this);
    auto *checkboxNormals = new QCheckBox(tr("Normals"), this);

    normalLength = new QLineEdit(this);
    normalLength->setFixedWidth(100);

    normalLength->setValidator( new QDoubleValidator(0, 100, 2, this) );

    mainLayout->addWidget(checkboxPoint);
    mainLayout->addWidget(checkboxLine);
    mainLayout->addWidget(checkboxTriangles);
    mainLayout->addWidget(checkboxNormals);
    mainLayout->addWidget(normalLength);

    connect(checkboxPoint, &QCheckBox::stateChanged, parent, &OpenGLWindow::setPoint);
    connect(checkboxLine, &QCheckBox::stateChanged, parent, &OpenGLWindow::setLine);
    connect(checkboxTriangles, &QCheckBox::stateChanged, parent, &OpenGLWindow::setTriangle);
    connect(checkboxNormals, &QCheckBox::stateChanged, opengl, [this](bool ok) {
        opengl->setNormalLength(normalLength->text().replace(',', '.').toFloat());
        opengl->setNormal(ok);
    });
    setLayout(mainLayout);
}
