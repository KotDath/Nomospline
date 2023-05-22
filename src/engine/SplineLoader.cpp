#include <QFile>
#include <QMessageBox>
#include <QOpenGLContext>
#include <QVector3D>
#include <QRegularExpression>

#include "engine/SplineLoader.h"

NURBS *SplineLoader::load(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);
    QVector<QVector4D> temp_controlPoints; // (x, y, z, w где w - вес)
    QVector<GLfloat> temp_knotU;
    QVector<GLfloat> temp_knotV;
    GLsizei vSize;

    return nullptr;
}