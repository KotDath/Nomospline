//
// Created by epida on 27.04.2023.
//
#include "structures/NURBS.h"
#include <QDebug>

Mesh *NURBS::evaluate()
{
    auto mesh = new Mesh();
    float N = 100, M = 100;
    float du = 1 / (N - 1), dv = 1 / (M - 1);
    mesh->vertices.reserve(N * M);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            float u = i * du;
            float v = j * dv;
            mesh->vertices.append(VertexData(getPoint(u, v), getNormal(u, v)));
            qDebug() << getNormal(u, v) << '\n';
        }
    }

    mesh->indices.reserve((N - 1) * (M - 1));

    for (int i = 0; i < N - 1; i++)
        for (int j = 0; j < M - 1; j++)
        {
            int offset = i * M + j;
            mesh->indices.append(i * M + j);
            mesh->indices.append((i + 1) * M + j);
            mesh->indices.append(i * M + j + 1);
            mesh->indices.append(i * M + j + 1);
            mesh->indices.append((i + 1) * M + j);
            mesh->indices.append((i + 1) * M + j + 1);
        }

    return mesh;
}

void NURBS::init()
{

}

int NURBS::findSpan(GLsizei degree, const QVector<GLfloat> &knots, GLfloat param)
{
    GLsizei n = knots.count() - degree - 2;
    assert(n >= 0);

    if (param > (knots[n + 1] - std::numeric_limits<float>::epsilon()))
    {
        return n;
    }
    if (param < (knots[degree] + std::numeric_limits<float>::epsilon()))
    {
        return degree;
    }

    GLsizei low = degree;
    GLsizei high = n + 1;
    auto mid = static_cast<GLsizei>(std::floor((low + high) / 2.0));
    while (param < knots[mid] || param >= knots[mid + 1])
    {
        if (param < knots[mid])
        {
            high = mid;
        } else
        {
            low = mid;
        }
        mid = static_cast<GLsizei>(std::floor((low + high) / 2.0));
    }
    return mid;
}

std::vector<GLfloat> NURBS::basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat> &knots, GLfloat u)
{
    std::vector<GLfloat> N;
    N.resize(deg + 1, 0);
    std::vector<GLfloat> left, right;
    left.resize(deg + 1, 0);
    right.resize(deg + 1, 0);
    GLfloat saved = 0.0, temp = 0.0;

    N[0] = 1.0;

    for (GLsizei j = 1; j <= static_cast<GLsizei>(deg); j++)
    {
        left[j] = (u - knots[span + 1 - j]);
        right[j] = knots[span + j] - u;
        saved = 0.0;
        for (GLsizei r = 0; r < j; r++)
        {
            temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

QVector3D NURBS::getPoint(GLfloat u, GLfloat v)
{
    int span_u = findSpan(uDegree, knotU, u);
    int span_v = findSpan(vDegree, knotV, v);

    auto Nu = basicFunctions(uDegree, span_u, knotU, u);
    auto Nv = basicFunctions(vDegree, span_v, knotV, v);

    QVector4D point;

    for (int l = 0; l <= vDegree; l++)
    {
        QVector4D temp;
        for (int k = 0; k <= uDegree; k++)
        {
            auto current = controlPoints[span_u - uDegree + k][span_v - vDegree + l];
            current.setX(current.x() * current.w());
            current.setY(current.y() * current.w());
            current.setZ(current.z() * current.w());
            temp += (Nu[k]) * current;
        }

        point += Nv[l] * temp;
    }

    QVector3D result;
    result.setX(point.x() / point.w());
    result.setY(point.y() / point.w());
    result.setZ(point.z() / point.w());
    return result;
}

std::pair<QVector3D, QVector3D> NURBS::getDerivatives(GLfloat u, GLfloat v)
{
    int span_u = findSpan(uDegree, knotU, u);
    int span_v = findSpan(vDegree, knotV, v);

    auto Nu = basicFunctions(uDegree, span_u, knotU, u);
    auto Nv = basicFunctions(vDegree, span_v, knotV, v);

    auto Ndu = basicFunctions(uDegree - 1, span_u, knotU, u);
    auto Ndv = basicFunctions(vDegree - 1, span_v, knotV, v);

    QVector4D point;

    for (int l = 0; l <= vDegree; l++)
    {
        QVector4D temp;
        for (int k = 0; k <= uDegree; k++)
        {
            auto current = controlPoints[span_u - uDegree + k][span_v - vDegree + l];
            current.setX(current.x() * current.w());
            current.setY(current.y() * current.w());
            current.setZ(current.z() * current.w());
            temp += (Nu[k]) * current;
        }

        point += Nv[l] * temp;
    }

    QVector3D A(point.x(), point.y(), point.z());
    float B = point.w();


    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    for (int j = 0; j <= vDegree; j++)
    {
        QVector4D temp;
        for (int i = 1; i <= uDegree; i++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i - 1][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = (tmp1 - tmp2) /
                           (knotU[span_u + i] - knotU[span_u - uDegree + i]);
            current.setX(current.x() * current.w());
            current.setY(current.y() * current.w());
            current.setZ(current.z() * current.w());
            temp += (Ndu[i - 1]) * current;
        }

        point += Nv[j] * temp;
    }

    QVector3D dA(point.x(), point.y(), point.z());
    float dB = point.w();

    auto der_u = (dA * B - dB * A) / (B * B);

    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    for (int i = 0; i <= uDegree; i++)
    {
        QVector4D temp;
        for (int j = 1; j <= vDegree; j++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j - 1];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = (tmp1 - tmp2) /
                           (knotV[span_v + j] - knotV[span_v - vDegree + j]);
            current.setX(current.x() * current.w());
            current.setY(current.y() * current.w());
            current.setZ(current.z() * current.w());
            temp += (Ndv[j - 1]) * current;
        }

        point += Nu[i] * temp;
    }

    dA.setX(point.x());
    dA.setY(point.y());
    dA.setZ(point.z());
    dB = point.w();

    auto der_v = (dA * B - dB * A) / (B * B);

    return std::make_pair(der_u, der_v);
}

QVector3D NURBS::getNormal(GLfloat u, GLfloat v)
{
    auto pair = getDerivatives(u, v);
    return QVector3D::crossProduct(pair.first, pair.second).normalized();
}
