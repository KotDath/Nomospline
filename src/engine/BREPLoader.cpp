#include <QFile>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFileInfo>
#include <QDir>

#include "engine/BREPLoader.h"

BREP *BREPLoader::load(const QString &path) {
    qDebug() << "Path: " << path;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);
    auto brep = new BREP();
    auto val = file.readAll();
    file.close();
    QJsonDocument d = QJsonDocument::fromJson(val);
    auto obj = d.object();
    auto surfaces = obj["surfaces"].toArray();

    auto rotationFactor = obj["rotation"].toArray();

    qreal x = rotationFactor.at(0).toDouble();
    qreal y = rotationFactor.at(1).toDouble();
    qreal z = rotationFactor.at(2).toDouble();

    QFileInfo fileInfo(path);
    for (const auto& surf : surfaces) {
        auto NURBSFilePath = surf.toObject().keys()[0];
        auto spl = splineLoader.load(fileInfo.dir().filePath(NURBSFilePath));
        brep->surfaces[spl] = {};
        for (const auto& curve : surf.toObject().value(NURBSFilePath).toObject().value("curves").toArray()) {
            auto crv = SplineLoader::loadCurve(fileInfo.dir().filePath( curve.toString()));
            auto polyline = SplineUtils::fitCurve(crv);
            brep->surfaces[spl].append(polyline);
        }

        qDebug() << fileInfo.dir().filePath(NURBSFilePath);
    }

    SplineUtils::rotate(brep, QVector3D(x, y, z));

    return brep;
}
