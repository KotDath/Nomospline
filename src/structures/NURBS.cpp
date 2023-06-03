//
// Created by epida on 27.04.2023.
//
#include "structures/NURBS.h"
#include <QDebug>

Mesh *NURBS::evaluate()
{
    auto mesh = new Mesh();
    float N = 10, M = 10;
    float du = (knotU.last() - knotU.first()) / (N - 1), dv = (knotV.last() - knotV.first()) / (M - 1);
    mesh->vertices.reserve(N * M);
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            float u = i * du;
            float v = j * dv;
            mesh->vertices.append(VertexData(getPoint(u, v), getNormal(u, v)));
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

    GLsizei low = degree;
    GLsizei high = n + 1;
    auto mid = static_cast<GLsizei>(std::floor((low + high) / 2.0));
    while (param < knots[mid] || param >= knots[mid + 1])
    {
        if (high - low <= 1)
        {
            return low;
        }
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

    int span_du = findSpan(uDegree - 1, knotU, u);
    int span_dv = findSpan(vDegree - 1, knotV, v);

    auto Ndu = basicFunctions(uDegree - 1, span_du, knotU, u);
    auto Ndv = basicFunctions(vDegree - 1, span_dv, knotV, v);

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
        for (int i = 0; i < uDegree; i++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i + 1][span_v - vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = uDegree * (tmp1 - tmp2) /
                           (knotU[span_u + i + 1] - knotU[span_u - uDegree + i + 1]);
            temp += (Ndu[i]) * current;
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
        for (int j = 0; j < vDegree; j++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j + 1];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = vDegree * (tmp1 - tmp2) /
                           (knotV[span_v + j + 1] - knotV[span_v - vDegree + j + 1]);
            temp += (Ndv[j]) * current;
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
    qDebug() << "du = " << pair.first << "dv = " << pair.second;
    return QVector3D::crossProduct(pair.first, pair.second).normalized();
}

QVector<QVector2D> NURBS::getInitialPoints(NURBS *otherSpline)
{
    return QVector<QVector2D>();
}

std::pair<QVector3D, QVector3D> NURBS::getDerivatives2(GLfloat u, GLfloat v)
{
    int span_u = findSpan(uDegree, knotU, u);
    int span_v = findSpan(vDegree, knotV, v);

    auto Nu = basicFunctions(uDegree, span_u, knotU, u);
    auto Nv = basicFunctions(vDegree, span_v, knotV, v);

    int span_du = findSpan(uDegree - 1, knotU, u);
    int span_dv = findSpan(vDegree - 1, knotV, v);

    auto Ndu = basicFunctions(uDegree - 1, span_du, knotU, u);
    auto Ndv = basicFunctions(vDegree - 1, span_dv, knotV, v);

    int span_ddu = findSpan(uDegree - 2, knotU, u);
    int span_ddv = findSpan(vDegree - 2, knotV, v);

    auto Nddu = basicFunctions(uDegree - 2, span_ddu, knotU, u);
    auto Nddv = basicFunctions(vDegree - 2, span_ddv, knotV, v);


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
        for (int i = 0; i < uDegree; i++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i + 1][span_v - vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = uDegree * (tmp1 - tmp2) /
                           (knotU[span_u + i + 1] - knotU[span_u - uDegree + i + 1]);
            temp += (Ndu[i]) * current;
        }

        point += Nv[j] * temp;
    }

    QVector3D dA(point.x(), point.y(), point.z());
    float dB = point.w();

    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    for (int j = 0; j <= vDegree; j++)
    {
        QVector4D temp;
        for (int i = 0; i < uDegree - 1; i++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i + 1][span_v - vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = (uDegree - 1) * (tmp1 * tmp1  - tmp2 * tmp2) /
                           (knotU[span_u + i] - knotU[span_u - uDegree + i + 1]);
            temp += (Nddu[i]) * current;
        }

        point += Nv[j] * temp;
    }

    QVector3D ddA(point.x(), point.y(), point.z());
    float ddB = point.w();


    auto der_u = ddA / B - 2 * dA * dB / (B * B) - A * (ddB / (B * B) - 2 * dB * dB / (B * B * B));

    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    for (int i = 0; i <= uDegree; i++)
    {
        QVector4D temp;
        for (int j = 0; j < vDegree; j++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j + 1];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = vDegree * (tmp1 - tmp2) /
                           (knotV[span_v + j + 1] - knotV[span_v - vDegree + j + 1]);
            temp += (Ndv[j]) * current;
        }

        point += Nu[i] * temp;
    }

    dA.setX(point.x());
    dA.setY(point.y());
    dA.setZ(point.z());
    dB = point.w();

    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    for (int i = 0; i <= uDegree; i++)
    {
        QVector4D temp;
        for (int j = 0; j < vDegree - 1; j++)
        {
            auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j + 1];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = (vDegree - 1) * (tmp1 * tmp1 - tmp2 * tmp2) /
                           (knotV[span_v + j] - knotV[span_v - vDegree + j + 1]);
            temp += (Nddv[j]) * current;
        }

        point += Nu[i] * temp;
    }

    ddA.setX(point.x());
    ddA.setY(point.y());
    ddA.setZ(point.z());
    ddB = point.w();

    auto der_v = ddA / B - 2 * dA * dB / (B * B) - A * (ddB / (B * B) - 2 * dB * dB / (B * B * B));

    return std::make_pair(der_u, der_v);
}
