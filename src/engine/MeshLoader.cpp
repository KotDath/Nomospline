#pragma once

#include <QFile>
#include <QMessageBox>
#include <QOpenGLContext>
#include <QVector3D>
#include <QRegularExpression>

#include "engine/MeshLoader.h"

Mesh* MeshLoader::load(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(0, "error", file.errorString());
    }

    QTextStream in(&file);

    auto* mesh = new Mesh();
    QVector<QVector3D> tmpVertices;
    QVector<QVector3D> tmpNormals;

    while (!in.atEnd())
    {
        QString line = in.readLine();
        if (line.startsWith("#"))
        {
            continue;
        }


        size_t linesCount;
        if (line.startsWith("l"))
        {
            size_t lineLocal = 0;
            GLushort previousValue = -1;
            QStringList words = line.split(" ");
            for (const QString &word: words)
            {
                auto value = word.toShort();
                if (value != 0)
                {
                    if (lineLocal != 0)
                    {
                        mesh->indices.append(previousValue - 1);
                        mesh->indices.append(value - 1);
                    }
                    previousValue = value;
                    ++lineLocal;
                }


            }
        }

        if (line.startsWith("f"))
        {
            QStringList words = line.split(" ");
            for (const QString &word: words)
            {
                auto value = word.split("/")[0].toShort();
                if (value != 0)
                {

                    mesh->indices.append(value - 1);

                }


            }
        }

        if (line.startsWith("v "))
        {
            QStringList words = line.split(" ");
            tmpVertices.append(QVector3D(
                    words[1].toFloat(),
                    words[2].toFloat(),
                    words[3].toFloat()
            ));
        }

        if (line.startsWith("vn "))
        {
            QStringList words = line.split(" ");
            tmpNormals.append(QVector3D(
                    words[1].toFloat(),
                    words[2].toFloat(),
                    words[3].toFloat()
            ));
        }
    }

    for (int i = 0; i < tmpVertices.count(); ++i)
    {
        if (i < tmpNormals.count()) {
            mesh->vertices.append(VertexData(tmpVertices[i], tmpNormals[i]));
        } else {
            mesh->vertices.append(VertexData(tmpVertices[i], QVector3D(0, 0, 0)));
        }

    }

    file.close();

    return mesh;
}

