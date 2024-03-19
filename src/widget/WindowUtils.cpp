#include "widget/WindowUtils.h"
#include "structures/SplineUtils.h"

QUrl WindowUtils::getHomeDirectory() {

    QUrl url{"file://" + QStandardPaths::writableLocation(QStandardPaths::HomeLocation)};
    qDebug() << url;
    return url;
}

void WindowUtils::importMesh(const QString &path) {
    qDebug() << path;
    QString absolutePath = QDir::current().absoluteFilePath(path);
    qDebug() << absolutePath;
    if (path.endsWith(".obj", Qt::CaseInsensitive))
    {
        bool isSpline = false;
        QFile file(path);
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
            auto *spline = splineLoader.load(path);
            scene->addSpline(spline);
        } else
        {
            auto *mesh = meshLoader.load(path);
            scene->addGeometry(mesh);
        }


    }
}

void WindowUtils::setForward() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(data->camera.getPosition().length(), 0, 0);
    window->update();
}

void WindowUtils::setBackward() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(-data->camera.getPosition().length(), 0, 0);
    window->update();
}

void WindowUtils::setUp() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(0, data->camera.getPosition().length(), 0);
    window->update();
}

void WindowUtils::setDown() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(0, -data->camera.getPosition().length(), 0);
    window->update();
}

void WindowUtils::setRight() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(0,0, data->camera.getPosition().length());
    window->update();
}

void WindowUtils::setLeft() {
    auto data = SceneData::getInstance();
    data->camera.moveTo(0,0, -data->camera.getPosition().length());
    window->update();
}

void WindowUtils::evaluate() {
    auto data = SceneData::getInstance();
    for (auto mesh: data->splineMeshes)
    {
        delete mesh;
    }
    data->splineMeshes.clear();

    for (const auto &spline:  data->splines)
    {
        data->splineMeshes.append(SplineUtils::evaluate(spline));
    }

    window->update();
}

void WindowUtils::calculateIntersection() {
    auto data = SceneData::getInstance();
    data->intersectionPoints = new Mesh();
    for (int i = 0; i < data->splines.length(); ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            auto intersection = SplineUtils::getInitialPoints(data->splines[j], data->splines[i]);
            qDebug() << "intersection: " << intersection;

            intersection = filterPoints(intersection);
            qDebug() << "intersection after filtering: " << intersection;
            for (auto k : intersection)
            {
                qDebug() << "Difference: " << (SplineUtils::getPoint(data->splines[i], k.z(), k.w()) -
                                               SplineUtils::getPoint(data->splines[j], k.x(), k.y())).length();
                data->intersectionPoints->vertices.append(SplineUtils::getPoint(data->splines[i], k.z(), k.w()));
                data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
            }


            for (auto point : intersection) {
                auto startPoint = point;

                if (QVector3D::crossProduct(SplineUtils::getNormal(data->splines[j], startPoint.x(), startPoint.y()),
                                            SplineUtils::getNormal(data->splines[i], startPoint.z(), startPoint.w())).length() < 0.001) {
                    qDebug() << "Found tangent intersection in point " << startPoint;
                }
                Timer timer("Plus side started", "Plus side finished");
                auto iterIntersection = SplineUtils::iterPointsPlus(data->splines[j], data->splines[i], startPoint);
                for (auto p : iterIntersection) {
                    data->intersectionPoints->vertices.append(p);
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
                timer = Timer("Minus side started", "Minus side finished");
                startPoint = point;
                iterIntersection = SplineUtils::iterPointsMinus(data->splines[j], data->splines[i], startPoint);
                for (auto p : iterIntersection) {
                    data->intersectionPoints->vertices.append(p);
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
            }
        }
    }
}

void WindowUtils::drawMeshes(bool checked) {
    auto data = SceneData::getInstance();
    data->isRenderMeshes = checked;
    window->update();
}

void WindowUtils::drawEvaluatedMeshes(bool checked) {
    auto data = SceneData::getInstance();
    data->isRenderEvaluate = checked;
    window->update();
}

void WindowUtils::drawSplines(bool checked) {
    auto data = SceneData::getInstance();
    data->isRenderSpline = checked;
    window->update();
}

QVector<QVector4D> WindowUtils::filterPoints(const QVector<QVector4D>& points)
{
    const float epsilon = 0.001;
    QVector<QVector4D> answer;
    for (const auto& point1 : points)
    {
        if (answer.isEmpty())
        {
            answer.append(point1);
        } else
        {
            float min = (point1 - answer[0]).length();
            for (const auto& point2 : answer)
            {
                float tmp = (point1 - point2).length();
                min = std::min(min, tmp);
            }

            if (min > epsilon)
            {
                answer.append(point1);
            }
        }

    }

    return answer;
}
