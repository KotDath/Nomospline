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

    auto spline = new NURBS();

    QVector<QVector4D> tmp_points;
    QVector<int> tmp_indices;
    float u_min, u_max, v_min, v_max;


    while (!in.atEnd())
    {
        QString line = in.readLine();
        if (line.startsWith("#"))
        {
            continue;
        }

        if (line.startsWith("v "))
        {
            QStringList words = line.split(" ");
            tmp_points.append({QVector4D(
                    words[1].toFloat(),
                    words[2].toFloat(),
                    words[3].toFloat(),
                    words[4].toFloat()
            )});
        }

        if (line.startsWith("parm u "))
        {
            QStringList words = line.split(" ");
            int tmp = 0;
            for (const QString &word: words)
            {
                if (tmp < 2) {
                    ++tmp;
                    continue;
                }

                auto value = word.split("/")[0].toFloat();
                spline->knotU.append(value);
                ++tmp;
            }
        }

        if (line.startsWith("parm v "))
        {
            QStringList words = line.split(" ");
            int tmp = 0;
            for (const QString &word: words)
            {
                if (tmp < 2) {
                    ++tmp;
                    continue;
                }

                auto value = word.split("/")[0].toFloat();
                spline->knotV.append(value);
                ++tmp;
            }
        }

        if (line.startsWith("deg "))
        {
            QStringList words = line.split(" ");
            spline->uDegree = words[1].toInt();
            spline->vDegree = words[2].toInt();
        }

        if (line.startsWith("surf "))
        {
            QStringList words = line.split(" ");
            int tmp = 0;
            for (const QString &word: words)
            {
                if (tmp == 1) {
                    u_min = word.toFloat();
                }

                if (tmp == 2) {
                    u_max = word.toFloat();
                }

                if (tmp == 3) {
                    v_min = word.toFloat();
                }

                if (tmp == 4) {
                    v_max = word.toFloat();
                }

                if (tmp > 4) {
                    tmp_indices.append(word.toFloat());
                }
                ++tmp;
            }
        }
    }

    int num_cp_u = spline->knotU.count() - spline->uDegree - 1;
    int num_cp_v = spline->knotV.count() - spline->vDegree - 1;

    spline->controlPoints.resize(num_cp_u);
    for (int i = 0; i < num_cp_u; ++i)
    {
        spline->controlPoints[i].resize(num_cp_v);
    }
    size_t num = 0;
    qDebug() << tmp_indices;
    qDebug() << tmp_points;
    qDebug() << num_cp_u << num_cp_v;
    for (int j = 0; j < num_cp_v; ++j)
    {
        for (int i = 0; i < num_cp_u; ++i)
        {
            spline->controlPoints[i][j] = tmp_points[tmp_indices[num] - 1];
            ++num;
        }
    }

    file.close();

    return spline;
}

NURBSCurve *SplineLoader::loadCurve(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);

    auto spline = new NURBSCurve();

    QVector<QVector3D> tmp_points;
    float u_min, u_max, v_min, v_max;


    while (!in.atEnd())
    {
        QString line = in.readLine();
        if (line.startsWith("looped"))
        {
            spline->isLooped = true;
        }

        if (line.startsWith("#"))
        {
            continue;
        }

        if (line.startsWith("v "))
        {
            QStringList words = line.split(" ");
            tmp_points.append({QVector3D(
                    words[1].toFloat(),
                    words[2].toFloat(),
                    words[3].toFloat()
            )});
        }

        if (line.startsWith("parm t "))
        {
            QStringList words = line.split(" ");
            int tmp = 0;
            for (const QString &word: words)
            {
                if (tmp < 2) {
                    ++tmp;
                    continue;
                }

                auto value = word.split("/")[0].toFloat();
                spline->knott.append(value);
                ++tmp;
            }
        }

        if (line.startsWith("deg "))
        {
            QStringList words = line.split(" ");
            spline->tDegree = words[1].toInt();
        }

        if (line.startsWith("surf "))
        {
            QStringList words = line.split(" ");
            int tmp = 0;
            for (const QString &word: words)
            {
                if (tmp == 1) {
                    u_min = word.toFloat();
                }

                if (tmp == 2) {
                    u_max = word.toFloat();
                }

                if (tmp == 3) {
                    v_min = word.toFloat();
                }

                if (tmp == 4) {
                    v_max = word.toFloat();
                }

                if (tmp > 4) {

                }
                ++tmp;
            }
        }
    }

    spline->controlPoints.resize(tmp_points.size());
    for (int i = 0; i < tmp_points.size(); ++i)
    {
        spline->controlPoints[i] = tmp_points[i];
    }

    file.close();

    qDebug() <<"Curve data: " << spline->controlPoints << spline->tDegree << spline->knott;
    return spline;
}
