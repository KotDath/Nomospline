#include "widget/WindowUtils.h"
#include "structures/SplineUtils.h"
#include "logger.h"

QUrl WindowUtils::getHomeDirectory() {

    QUrl url{"file://" + QStandardPaths::writableLocation(QStandardPaths::HomeLocation)};
    qDebug() << url;
    return url;
}

void WindowUtils::importMesh(const QString &path) {
    qDebug() << path;
    QString absolutePath = QDir::current().absoluteFilePath(path);
    qDebug() << absolutePath;
    if (path.endsWith(".obj", Qt::CaseInsensitive)) {
        bool isSpline = false;
        QFile file(path);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream stream(&file);
            while (!stream.atEnd()) {
                QString line = stream.readLine();
                if (line.contains("cstype rat bspline", Qt::CaseInsensitive)) {
                    isSpline = true;
                    break;
                }
            }
            file.close();
        }


        if (isSpline) {
            auto *spline = splineLoader.load(path);
            scene->addSpline(spline);
        } else {
            auto *mesh = meshLoader.load(path);
            scene->addGeometry(mesh);
        }


    }
}

void WindowUtils::setForward() {

    data->camera.moveTo(data->camera.getPosition().length(), 0, 0);
    window->update();
}

void WindowUtils::setBackward() {

    data->camera.moveTo(-data->camera.getPosition().length(), 0, 0);
    window->update();
}

void WindowUtils::setUp() {

    data->camera.moveTo(0, data->camera.getPosition().length(), 0);
    window->update();
}

void WindowUtils::setDown() {

    data->camera.moveTo(0, -data->camera.getPosition().length(), 0);
    window->update();
}

void WindowUtils::setRight() {

    data->camera.moveTo(0, 0, data->camera.getPosition().length());
    window->update();
}

void WindowUtils::setLeft() {

    data->camera.moveTo(0, 0, -data->camera.getPosition().length());
    window->update();
}

void WindowUtils::evaluate() {
    data->evalutaionBlocker = true;
    auto future = QtConcurrent::run(asyncEvaluate); // for class method
    QObject::connect(&watcherEvalutation, &QFutureWatcher<void>::finished, [&]() {
        qDebug() << "Async function completed!";
        // Execute another code block here
        data->evalutaionBlocker = false;
        window->update();
    });
    watcherEvalutation.setFuture(future);
}

void WindowUtils::calculateIntersection() {

    data->intersectionBlocker = true;
    auto future = QtConcurrent::run(asyncIntersection); // for class method
    QObject::connect(&watcherIntersection, &QFutureWatcher<void>::finished, [&]() {
        qDebug() << "Async function completed!";
        // Execute another code block here
        data->intersectionBlocker = false;
        window->update();
    });
    watcherIntersection.setFuture(future);
}

void WindowUtils::drawMeshes(bool checked) {

    data->isRenderMeshes = checked;
    window->update();
}

void WindowUtils::drawEvaluatedMeshes(bool checked) {

    data->isRenderEvaluate = checked;
    window->update();
}

void WindowUtils::drawSplines(bool checked) {

    data->isRenderSpline = checked;
    window->update();
}

WindowUtils::WindowUtils() : QObject() {
    data = SceneData::getInstance();
}

void WindowUtils::clear() {
    for (const auto &obj: data->splines) {
        delete obj;
    }

    data->splines.clear();

    for (const auto &obj: data->meshes) {
        delete obj;
    }

    data->meshes.clear();

    for (const auto &obj: data->splineMeshes) {
        delete obj;
    }

    data->splineMeshes.clear();

    delete data->intersectionPoints;

    window->update();
}

void WindowUtils::importBREP(const QString &path) {
    qDebug() << path;
    QString absolutePath = QDir::current().absoluteFilePath(path);
    qDebug() << absolutePath;
    if (path.endsWith(".json", Qt::CaseInsensitive)) {
        auto *brep = brepLoader.load(path);
        scene->addBREP(brep);


    }
}

QUrl WindowUtils::getDefaultSplineFolder() {
    QUrl url{"file:///home/kotdath/Documents/qt_projects/Nomospline/examples"};
    qDebug() << url;
    return url;

}

QUrl WindowUtils::getDefaultBREPFolder() {
    QUrl url{"file:///home/kotdath/Documents/qt_projects/Nomospline/examples/BREP"};
    qDebug() << url;
    return url;
}

void WindowUtils::calculateDiff() {
    data->intersectionBlocker = true;
    auto future = QtConcurrent::run(asyncDiff); // for class method
    QObject::connect(&watcherIntersection, &QFutureWatcher<void>::finished, [&]() {
        qDebug() << "Async function completed!";
        // Execute another code block here
        data->intersectionBlocker = false;
        window->update();
    });
    watcherIntersection.setFuture(future);
}

void asyncDiff() {
    auto data = SceneData::getInstance();
    auto logger = Logger::getInstance();

    for (int i = 0; i < data->breps.length(); ++i) {
        for (int j = 0; j < i; ++j) {
            Tesselator::diff(data->breps[j], data->breps[i]);
        }
    }

    return;
}

void asyncIntersection() {
    auto data = SceneData::getInstance();
    auto logger = Logger::getInstance();

    for (int i = 0; i < data->breps.length(); ++i) {
        for (int j = 0; j < i; ++j) {
            Tesselator::intersection(data->breps[j], data->breps[i]);
        }
    }

    return;

    /*
    data->intersectionPoints = new Mesh();
    for (int i = 0; i < data->splines.length(); ++i) {
        for (int j = 0; j < i; ++j) {

            if (!SplineUtils::isBBIntersected(data->splines[j], data->splines[i])) {
                qDebug() << "Bounding boxes not intersected. Exit.";
                continue;
            }


            logger->write(data->splines[j]->knotU.first(), data->splines[j]->knotU.last(),
                          data->splines[j]->knotV.first(), data->splines[j]->knotV.last());

            auto intersection = SplineUtils::getInitialPoints(data->splines[j], data->splines[i]);
            qDebug() << "intersection: " << intersection;

            intersection = filterPoints(intersection);
            qDebug() << "intersection after filtering: " << intersection;
            for (auto k: intersection) {
                qDebug() << "Difference: " << (SplineUtils::getPoint(data->splines[i], k.z(), k.w()) -
                                               SplineUtils::getPoint(data->splines[j], k.x(), k.y())).length();
                data->intersectionPoints->vertices.append(SplineUtils::getPoint(data->splines[i], k.z(), k.w()));
                data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
            }


            for (auto point: intersection) {
                auto startPoint = point;

                if (QVector3D::crossProduct(SplineUtils::getNormal(data->splines[j], startPoint.x(), startPoint.y()),
                                            SplineUtils::getNormal(data->splines[i], startPoint.z(),
                                                                   startPoint.w())).length() < 0.001) {
                    qDebug() << "Found tangent intersection in point " << startPoint;
                }
                Timer timer("Plus side started", "Plus side finished");
                auto iterIntersection = SplineUtils::iterPointsPlus(data->splines[j], data->splines[i], startPoint);
                for (auto p: iterIntersection) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(data->splines[j], p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
                timer = Timer("Minus side started", "Minus side finished");
                startPoint = point;
                iterIntersection = SplineUtils::iterPointsMinus(data->splines[j], data->splines[i], startPoint);
                for (auto p: iterIntersection) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(data->splines[j], p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
            }
        }
    }
     */

    logger->deployData("abobus.txt");
}


void asyncEvaluate() {
    auto data = SceneData::getInstance();
    for (auto mesh: data->splineMeshes) {
        delete mesh;
    }
    data->splineMeshes.clear();

    for (const auto &spline: data->splines) {
        data->splineMeshes.append(SplineUtils::evaluate(spline));
    }

    for (const auto &brep: data->breps) {
        //brep->isInsideBREP({2, 2, 0.4});

        for (const auto &spline: brep->surfaces.keys()) {
            qDebug() << "Center: " << SplineUtils::getPoint(spline, 0.5, 0.5) << "Normal: "
                     << SplineUtils::getNormal(spline, 0.5, 0.5);
            QVector<QVector<QVector2D>> limits;
            for (const auto &curv: brep->surfaces[spline]) {
                limits.append(curv->sequencePoints);
            }

            if (limits.empty()) {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, true));
            } else {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, false));
            }

        }
    }
}