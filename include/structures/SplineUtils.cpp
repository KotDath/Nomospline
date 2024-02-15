//
// Created by kotdath on 2/15/24.
//

#include <QMatrix4x4>
#include "SplineUtils.h"

float deltaAlpha = M_PI / 15;
float stepMin = 0.01;
float stepMax = 0.1;

class NotBoundException : public std::exception {
public:
    char * what () {
        return "Point not bound";
    }
};

float determinant(QMatrix3x3 mat)
{
    float det = 0;
    for (int i = 0; i < 3; i++)
    {
        det += mat(0, i) * (mat(1, (i + 1) % 3) * mat(2, (i + 2) % 3) - mat(1, (i + 2) % 3) * mat(2, (i + 1) % 3));
    }
    return det;
}

QMatrix3x3 invertMatrix3D(const QMatrix3x3 &matrix)
{
    float det = determinant(matrix);
    if (std::abs(det) < 0.0001) // Check for singularity
        return QMatrix3x3();

    // Calculate matrix of cofactors
    QMatrix3x3 cofactors;
    cofactors(0, 0) = matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1);
    cofactors(0, 1) = matrix(1, 2) * matrix(2, 0) - matrix(1, 0) * matrix(2, 2);
    cofactors(0, 2) = matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0);
    cofactors(1, 0) = matrix(0, 2) * matrix(2, 1) - matrix(0, 1) * matrix(2, 2);
    cofactors(1, 1) = matrix(0, 0) * matrix(2, 2) - matrix(0, 2) * matrix(2, 0);
    cofactors(1, 2) = matrix(0, 1) * matrix(2, 0) - matrix(0, 0) * matrix(2, 1);
    cofactors(2, 0) = matrix(0, 1) * matrix(1, 2) - matrix(0, 2) * matrix(1, 1);
    cofactors(2, 1) = matrix(0, 2) * matrix(1, 0) - matrix(0, 0) * matrix(1, 2);
    cofactors(2, 2) = matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);

    // Build the adjugate matrix
    cofactors = cofactors.transposed();

    // Multiply the adjugate matrix by the reciprocal of the determinant
    return cofactors * (1 / det);
}

QMatrix4x4 invertMatrix4D(const QMatrix4x4 &matrix)
{
    return matrix.inverted();
}


Mesh *SplineUtils::evaluate(NURBS *spline) {
    Timer timer("Evaluate mesh started", "Evaluate mesh finished");
    auto mesh = new Mesh();
    float N = 100, M = 100;
    float du = (spline->knotU.last() - spline->knotU.first()) / (N - 1), dv = (spline->knotV.last() -
            spline->knotV.first()) / (M - 1);
    mesh->vertices.reserve(N * M);
    int k = 0;
    for (int i = 0; i < N; ++i)
    {
        float u = i * du;
        for (int j = 0; j < M; ++j)
        {
            float v = j * dv;
            if (u > 1 && k == 0)
            {
                auto p1 = getPoint(spline, u - du, v);
                auto p2 = getPoint(spline, u, v);
                auto p3 = getPoint(spline, u + du, v);
                qDebug() << "Points: " << p1 << p2 << p3;
                qDebug() << "Accurate derivative: " << u << v << getDerivatives(spline, u, v).first << "Approximation: "
                         << (p3 - p1) / (du * 2);
                qDebug() << "Accurate 2 derivative: " << u << v << getDerivatives2(spline, u, v).first << "Approximation: "
                         << (p3 - 2 * p2 + p1) / (du * du);
                ++k;
            }


            mesh->vertices.append(VertexData(getPoint(spline, u, v), getNormal(spline, u, v)));
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

QVector3D SplineUtils::getPoint(NURBS *spline, GLfloat u, GLfloat v) {
    int span_u = findSpan(spline->uDegree, spline->knotU, u);
    int span_v = findSpan(spline->vDegree, spline->knotV, v);

    auto Nu = basicFunctions(spline->uDegree, span_u, spline->knotU, u);
    auto Nv = basicFunctions(spline->vDegree, span_v, spline->knotV, v);

    QVector4D point;

    for (int l = 0; l <= spline->vDegree; l++)
    {
        QVector4D temp;
        for (int k = 0; k <= spline->uDegree; k++)
        {
            auto current = spline->controlPoints[span_u - spline->uDegree + k][span_v - spline->vDegree + l];
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

int SplineUtils::findSpan(GLsizei degree, const QVector<GLfloat> &knots, GLfloat param) {
    GLsizei n = knots.count() - degree - 2;
    assert(n >= 0);
    auto a = knots[n + 1];
    if (param >= (knots[n + 1] - std::numeric_limits<float>::epsilon()))
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

std::vector<GLfloat> SplineUtils::basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat> &knots, GLfloat u) {
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

GLfloat
SplineUtils::getStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                           const QVector3D &t) {
    float res = abs(QVector3D::dotProduct(ur1, t));
    float error = 0.001f;
    float min = std::max(res, error);


    res = abs(QVector3D::dotProduct(vr2, t));
    if (res > error && res < min)
    {
        min = res;
    }

    res = abs(QVector3D::dotProduct(as1, t));
    if (res > error && res < min)
    {
        min = res;
    }

    res = abs(QVector3D::dotProduct(bs2, t));
    if (res > error && res < min)
    {
        min = res;
    }

    return min / t.length();
}

int SplineUtils::getDirectionStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1,
                                        const QVector3D &bs2, const QVector3D &t) {
    float res = abs(QVector3D::dotProduct(ur1, t));
    float error = 0.01f;
    float max = std::max(res, error);
    int maxi = 0;

    res = abs(QVector3D::dotProduct(vr2, t));
    if (res > error && res >= max)
    {
        max = res;
        maxi = 1;
    }

    res = abs(QVector3D::dotProduct(as1, t));
    if (res > error && res >= max)
    {
        max = res;
        maxi = 2;
    }

    res = abs(QVector3D::dotProduct(bs2, t));
    if (res > error && res >= max)
    {
        max = res;
        maxi = 3;
    }

    return maxi;
}

QVector<QVector4D> SplineUtils::curveToSurfaceIntersectionU(NURBS *spline1, GLfloat u, NURBS *spline2) {
    QVector<QVector4D> intersections;

    GLfloat tempV = spline1->knotV.first();

    QMatrix2x2 g;
    QMatrix2x2 tmpMatrix;

    while (tempV < spline1->knotV.last())
    {
        auto deltaV = getAdaptiveStepCurveV(spline1, u, tempV);
        GLfloat tempA = spline2->knotU.first();
        GLfloat tempB = spline2->knotV.first();

        auto c0 = getPoint(spline1, u, tempV);
        auto c = getDerivatives(spline1, u, tempV).second;

        while (tempA < spline2->knotU.last())
        {
            auto deltaA = getAdaptiveStepU(spline2, tempA, tempB);

            while (tempB < spline2->knotV.last())
            {
                auto deltaB = getAdaptiveStepV(spline2, tempA, tempB);

                auto der1 = getDerivatives(spline2, tempA, tempB); // s1, s2
                auto normal = QVector3D::crossProduct(der1.first, der1.second).normalized();
                auto s0 = getPoint(spline2, tempA, tempB);

                auto t0 = QVector3D::dotProduct(normal, s0 - c0) / QVector3D::dotProduct(normal, c);
                auto ps = c0 + t0 * c;


                tmpMatrix(0, 0) = QVector3D::dotProduct(der1.first, der1.first);
                tmpMatrix(0, 1) = QVector3D::dotProduct(der1.first, der1.second);
                tmpMatrix(1, 0) = QVector3D::dotProduct(der1.second, der1.first);
                tmpMatrix(1, 1) = QVector3D::dotProduct(der1.second, der1.second);

                auto tmpDot = tmpMatrix(0, 0) * tmpMatrix(1, 1) - tmpMatrix(1, 0) * tmpMatrix(0, 1);


                g(0, 0) = tmpMatrix(1, 1) / tmpDot;
                g(0, 1) = -tmpMatrix(0, 1) / tmpDot;
                g(1, 0) = -tmpMatrix(1, 0) / tmpDot;
                g(1, 1) = tmpMatrix(0, 0) / tmpDot;

                auto u0 = g(0, 0) * QVector3D::dotProduct(der1.first, ps - s0) +
                          g(0, 1) * QVector3D::dotProduct(der1.second, ps - s0);

                auto v0 = g(1, 0) * QVector3D::dotProduct(der1.first, ps - s0) +
                          g(1, 1) * QVector3D::dotProduct(der1.second, ps - s0);


                // t0, u0, v0 - начальные приближения метода Ньютона для решения системы
                if (abs(t0) <= deltaV && abs(u0) <= deltaA && abs(v0) <= deltaB) // вблизи точки пересечения
                {
                    QVector3D zeroSolution(tempV + t0, tempA + u0, tempB + v0); // начальное приближение

                    // дальше идёт решение системы численным методом(например Ньютона)
                    try
                    {
                        auto solution = NewtonSolution3D(spline1, spline2, zeroSolution, 0, u);
                        QVector4D insertSolution{u, solution.x(), solution.y(), solution.z()};
                        auto delta = (getPoint(spline1, u, solution.x()) - getPoint(spline2, solution.y(), solution.z())).length();
                        if (isBound(spline1, spline2, insertSolution) && delta < 0.001) {
                            intersections.append(insertSolution);
                        }
                    }catch (NotBoundException& e)
                    {
                        qDebug() << e.what();
                    }


                }

                tempB += deltaB;
            }

            tempB = spline2->knotV.first();
            tempA += deltaA;
        }

        tempB = spline2->knotV.first();
        tempA = spline2->knotU.first();
        tempV += deltaV;

    }


    return intersections;
}

std::pair<QVector3D, QVector3D> SplineUtils::getDerivatives(NURBS *spline, GLfloat u, GLfloat v) {
    int span_u = findSpan(spline->uDegree, spline->knotU, u);
    int span_v = findSpan(spline->vDegree, spline->knotV, v);

    auto Nu = basicFunctions(spline->uDegree, span_u, spline->knotU, u);
    auto Nv = basicFunctions(spline->vDegree, span_v, spline->knotV, v);

    int span_du = span_u;
    int span_dv = span_v;

    auto Ndu = basicFunctions(spline->uDegree - 1, span_du, spline->knotU, u);
    auto Ndv = basicFunctions(spline->vDegree - 1, span_dv, spline->knotV, v);

    QVector4D point;

    for (int l = 0; l <= spline->vDegree; l++)
    {
        QVector4D temp;
        for (int k = 0; k <= spline->uDegree; k++)
        {
            auto current = spline->controlPoints[span_u - spline->uDegree + k][span_v - spline->vDegree + l];
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

    for (int j = 0; j <= spline->vDegree; j++)
    {
        QVector4D temp;
        for (int i = 0; i < spline->uDegree; i++)
        {
            auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i + 1][span_v - spline->vDegree + j];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = spline->uDegree * (tmp1 - tmp2) /
                           (spline->knotU[span_u + i + 1] - spline->knotU[span_u - spline->uDegree + i + 1]);
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

    for (int i = 0; i <= spline->uDegree; i++)
    {
        QVector4D temp;
        for (int j = 0; j < spline->vDegree; j++)
        {
            auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j + 1];
            tmp1.setX(tmp1.x() * tmp1.w());
            tmp1.setY(tmp1.y() * tmp1.w());
            tmp1.setZ(tmp1.z() * tmp1.w());

            auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
            tmp2.setX(tmp2.x() * tmp2.w());
            tmp2.setY(tmp2.y() * tmp2.w());
            tmp2.setZ(tmp2.z() * tmp2.w());

            auto current = spline->vDegree * (tmp1 - tmp2) /
                           (spline->knotV[span_v + j + 1] - spline->knotV[span_v - spline->vDegree + j + 1]);
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

QVector3D SplineUtils::getNormal(NURBS *spline, GLfloat u, GLfloat v) {
    auto pair = getDerivatives(spline, u, v);
    return QVector3D::crossProduct(pair.first, pair.second).normalized();
}

std::pair<QVector3D, QVector3D> SplineUtils::getDerivatives2(NURBS *spline, GLfloat u, GLfloat v) {
    int span_u = findSpan(spline->uDegree, spline->knotU, u);
    int span_v = findSpan(spline->vDegree, spline->knotV, v);

    auto Nu = basicFunctions(spline->uDegree, span_u, spline->knotU, u);
    auto Nv = basicFunctions(spline->vDegree, span_v, spline->knotV, v);

    int span_du = span_u;
    int span_dv = span_v;

    auto Ndu = basicFunctions(spline->uDegree - 1, span_du, spline->knotU, u);
    auto Ndv = basicFunctions(spline->vDegree - 1, span_dv, spline->knotV, v);

    int span_ddu = span_u;
    int span_ddv = span_v;

    std::vector<float> Nddu, Nddv;

    QVector3D der_v(0, 0, 0);
    QVector3D der_u(0, 0, 0);


    QVector4D point;

    for (int l = 0; l <= spline->vDegree; l++)
    {
        QVector4D temp;
        for (int k = 0; k <= spline->uDegree; k++)
        {
            auto current = spline->controlPoints[span_u - spline->uDegree + k][span_v - spline->vDegree + l];
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


    if (spline->uDegree >= 2) {
        for (int j = 0; j <= spline->vDegree; j++)
        {
            QVector4D temp;
            for (int i = 0; i < spline->uDegree; i++)
            {
                auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i + 1][span_v - spline->vDegree + j];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto current = spline->uDegree * (tmp1 - tmp2) /
                               (spline->knotU[span_u + i + 1] - spline->knotU[span_u - spline->uDegree + i + 1]);
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

        Nddu = basicFunctions(spline->uDegree - 2, span_ddu, spline->knotU, u);

        for (int j = 0; j <= spline->vDegree; j++)
        {
            QVector4D temp;
            for (int i = 0; i < spline->uDegree - 1; i++)
            {
                auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i + 2][span_v - spline->vDegree + j];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i + 1][span_v - spline->vDegree + j];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto tmp3 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
                tmp3.setX(tmp3.x() * tmp3.w());
                tmp3.setY(tmp3.y() * tmp3.w());
                tmp3.setZ(tmp3.z() * tmp3.w());

                auto pi = spline->uDegree * (tmp1 - tmp2) / (spline->knotU[span_u + i + 2] - spline->knotU[span_u - spline->uDegree + i + 2]);
                auto pi_1 = spline->uDegree * (tmp2 - tmp3) / (spline->knotU[span_u + i + 1] - spline->knotU[span_u - spline->uDegree + i + 1]);

                auto current = (spline->uDegree - 1) * (pi - pi_1) /
                               (spline->knotU[span_u + i + 1] - spline->knotU[span_u - spline->uDegree + i + 2]);
                temp += (Nddu[i]) * current;
            }

            point += Nv[j] * temp;
        }

        QVector3D ddA(point.x(), point.y(), point.z());
        float ddB = point.w();

        der_u = ddA / B - 2 * dA * dB / (B * B) - A * (ddB / (B * B) - 2 * dB * dB / (B * B * B));
    }





    point.setX(0);
    point.setY(0);
    point.setZ(0);
    point.setW(0);

    if (spline->vDegree >= 2) {

        for (int i = 0; i <= spline->uDegree; i++) {
            QVector4D temp;
            for (int j = 0; j < spline->vDegree; j++) {
                auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j + 1];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto current = spline->vDegree * (tmp1 - tmp2) /
                               (spline->knotV[span_v + j + 1] - spline->knotV[span_v - spline->vDegree + j + 1]);
                temp += (Ndv[j]) * current;
            }

            point += Nu[i] * temp;
        }

        QVector3D dA(point.x(), point.y(), point.z());
        float dB = point.w();

        point.setX(0);
        point.setY(0);
        point.setZ(0);
        point.setW(0);

        Nddv = basicFunctions(spline->vDegree - 2, span_ddv, spline->knotV, v);

        for (int i = 0; i <= spline->uDegree; i++) {
            QVector4D temp;
            for (int j = 0; j < spline->vDegree - 1; j++) {
                auto tmp1 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j + 2];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j + 1];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto tmp3 = spline->controlPoints[span_u - spline->uDegree + i][span_v - spline->vDegree + j];
                tmp3.setX(tmp3.x() * tmp3.w());
                tmp3.setY(tmp3.y() * tmp3.w());
                tmp3.setZ(tmp3.z() * tmp3.w());

                auto pi = spline->vDegree * (tmp1 - tmp2) / (spline->knotV[span_v + j + 2] - spline->knotV[span_v - spline->vDegree + j + 2]);
                auto pi_1 = spline->vDegree * (tmp2 - tmp3) / (spline->knotV[span_v + j + 1] - spline->knotV[span_v - spline->vDegree + j + 1]);

                auto current = (spline->vDegree - 1) * (pi - pi_1) /
                               (spline->knotV[span_v + j + 1] - spline->knotV[span_v - spline->vDegree + j + 2]);

                temp += (Nddv[j]) * current;
            }

            point += Nu[i] * temp;
        }

        QVector3D ddA(point.x(), point.y(), point.z());
        float ddB = point.w();

        der_v = ddA / B - 2 * dA * dB / (B * B) - A * (ddB / (B * B) - 2 * dB * dB / (B * B * B));
    }


    return std::make_pair(der_u, der_v);
}

QVector3D SplineUtils::getDerivativesMixed(NURBS *spline, GLfloat u, GLfloat v) {
    GLfloat du = 0.01;
    auto der1 = getDerivatives(spline, u + du, v);
    auto der2 = getDerivatives(spline, u - du, v);
    return (der1.second - der2.second) / (2 * du);
}

QVector<QVector4D> SplineUtils::curveToSurfaceIntersectionV(NURBS *spline1, GLfloat v, NURBS *spline2) {
    QVector<QVector4D> intersections;

    GLfloat tempU = spline1->knotU.first();

    QMatrix2x2 g;
    QMatrix2x2 tmpMatrix;

    while (tempU < spline1->knotU.last())
    {
        auto deltaU = getAdaptiveStepCurveU(spline1, tempU, v);
        GLfloat tempA = spline2->knotU.first();
        GLfloat tempB = spline2->knotV.first();

        auto c0 = getPoint(spline1, tempU, v);
        auto c = getDerivatives(spline1, tempU, v).first;

        while (tempA < spline2->knotU.last())
        {
            auto deltaA = getAdaptiveStepU( spline2, tempA, tempB);

            while (tempB <  spline2->knotV.last())
            {
                auto deltaB = getAdaptiveStepV(spline2, tempA, tempB);

                auto der1 = getDerivatives(spline2, tempA, tempB); // s1, s2
                auto normal = QVector3D::crossProduct(der1.first, der1.second).normalized();
                auto s0 = getPoint(spline2, tempA, tempB);

                auto t0 = QVector3D::dotProduct(normal, s0 - c0) / QVector3D::dotProduct(normal, c);
                auto ps = c0 + t0 * c;


                tmpMatrix(0, 0) = QVector3D::dotProduct(der1.first, der1.first);
                tmpMatrix(0, 1) = QVector3D::dotProduct(der1.first, der1.second);
                tmpMatrix(1, 0) = QVector3D::dotProduct(der1.second, der1.first);
                tmpMatrix(1, 1) = QVector3D::dotProduct(der1.second, der1.second);

                auto tmpDot = tmpMatrix(0, 0) * tmpMatrix(1, 1) - tmpMatrix(1, 0) * tmpMatrix(0, 1);


                g(0, 0) = tmpMatrix(1, 1) / tmpDot;
                g(0, 1) = -tmpMatrix(0, 1) / tmpDot;
                g(1, 0) = -tmpMatrix(1, 0) / tmpDot;
                g(1, 1) = tmpMatrix(0, 0) / tmpDot;

                auto u0 = g(0, 0) * QVector3D::dotProduct(der1.first, ps - s0) +
                          g(0, 1) * QVector3D::dotProduct(der1.second, ps - s0);

                auto v0 = g(1, 0) * QVector3D::dotProduct(der1.first, ps - s0) +
                          g(1, 1) * QVector3D::dotProduct(der1.second, ps - s0);


                // t0, u0, v0 - начальные приближения метода Ньютона для решения системы

                if (abs(t0) <= deltaU && abs(u0) <= deltaA && abs(v0) <= deltaB) // вблизи точки пересечения
                {
                    QVector3D zeroSolution(tempU + t0, tempA + u0, tempB + v0); // начальное приближение

                    // дальше идёт решение системы численным методом(например Ньютона)


                    try
                    {
                        auto solution = NewtonSolution3D(spline1, spline2, zeroSolution, 1, v);
                        QVector4D insertSolution{solution.x(), v, solution.y(), solution.z()};
                        auto delta = (getPoint(spline1, solution.x(), v) - getPoint(spline2, solution.y(), solution.z())).length();
                        if (isBound(spline1, spline2, insertSolution) && delta < 0.001)
                        {
                            intersections.append(insertSolution);
                        }
                    } catch (NotBoundException& e)
                    {
                        qDebug() << e.what();
                    }

                }

                tempB += deltaB;
            }

            tempB = spline2->knotV.first();
            tempA += deltaA;
        }

        tempB = spline2->knotV.first();
        tempA = spline2->knotU.first();
        tempU += deltaU;

    }


    return intersections;
}

GLfloat SplineUtils::getAdaptiveStepU(NURBS *spline, GLfloat u, GLfloat v) {
    auto der1 = getDerivatives(spline, u, v);
    auto der2 = getDerivatives2(spline, u, v);
    auto normal = getNormal(spline, u, v);

    if (der2.first.length() < 0.001)
    {
        return stepMin;
    }

    auto deltaU = deltaAlpha * der1.first.length() / abs(QVector3D::dotProduct(normal, der2.first));
    if (deltaU < stepMin)
    {
        return stepMin;
    }

    if (deltaU > stepMax)
    {
        return stepMax;
    }
    return deltaU;
}

GLfloat SplineUtils::getAdaptiveStepV(NURBS *spline, GLfloat u, GLfloat v) {
    auto der1 = getDerivatives(spline, u, v);
    auto der2 = getDerivatives2(spline, u, v);
    auto normal = getNormal(spline, u, v);

    if (der2.second.length() < 0.001)
    {
        return stepMin;
    }

    auto deltaV = deltaAlpha * der1.second.length() / abs(QVector3D::dotProduct(normal, der2.second));
    if (deltaV < stepMin)
    {
        return stepMin;
    }

    if (deltaV > stepMax)
    {
        return stepMax;
    }

    return deltaV;
}

GLfloat SplineUtils::getAdaptiveStepCurveU(NURBS *spline, GLfloat u, GLfloat v) {
    auto der1 = getDerivatives(spline, u, v);
    auto der2 = getDerivatives2(spline, u, v);

    if (der2.first.length() < 0.001)
    {
        return stepMin;
    }

    auto deltaT = deltaAlpha * der1.first.length() * der1.first.length() /
                  (QVector3D::crossProduct(der1.first, der2.first).length());

    if (deltaT < stepMin)
    {
        return stepMin;
    }

    if (deltaT > stepMax)
    {
        return stepMax;
    }
    return deltaT;
}

GLfloat SplineUtils::getAdaptiveStepCurveV(NURBS *spline, GLfloat u, GLfloat v) {
    auto der1 = getDerivatives(spline, u, v);
    auto der2 = getDerivatives2(spline, u, v);

    if (der2.second.length() < 0.001)
    {
        return stepMin;
    }

    auto deltaT = deltaAlpha * der1.second.length() * der1.second.length() /
                  (QVector3D::crossProduct(der1.second, der2.second).length());
    if (deltaT < stepMin)
    {
        return stepMin;
    }

    if (deltaT > stepMax)
    {
        return stepMax;
    }
    return deltaT;
}

QVector<QVector4D> SplineUtils::getInitialPoints(NURBS *spline1, NURBS *spline2) {
    QVector<QVector4D> answer;

    float stepU = 2;
    float stepV = 2;

    auto du1 = (spline1->knotU.last() - spline1->knotU.first()) / stepU;
    auto dv1 = (spline1->knotV.last() - spline1->knotV.first()) / stepV;
    auto du2 = (spline2->knotU.last() - spline2->knotU.first()) / stepU;
    auto dv2 = (spline2->knotV.last() - spline2->knotV.first()) / stepV;

    for (auto U = spline1->knotU.first(); U < spline1->knotU.last(); U += du1)
    {
        answer += curveToSurfaceIntersectionU(spline1, U, spline2);

        qDebug() << "curve to surface u: ";
        for (const auto& ans : answer)
        {
            qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
        }
    }

    answer += curveToSurfaceIntersectionU(spline1, spline1->knotU.last(), spline2);

    qDebug() << "curve to surface u: ";
    for (const auto& ans : answer)
    {
        qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
    }

    for (auto V = spline1->knotV.first(); V <= spline1->knotV.last(); V += dv1)
    {
        answer += curveToSurfaceIntersectionV(spline1, V, spline2);

        qDebug() << "curve to surface v: ";
        for (const auto& ans : answer)
        {
            qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
        }
    }

    answer += curveToSurfaceIntersectionV(spline1, spline1->knotV.last(), spline2);

    qDebug() << "curve to surface v: ";
    for (const auto& ans : answer)
    {
        qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
    }

    for (auto U = spline2->knotU.first(); U < spline2->knotU.last(); U += du2)
    {

        auto tmp = curveToSurfaceIntersectionU(spline2, U, spline1);
        for (auto& elem : tmp)
        {
            float tempX = elem.x();
            float tempY = elem.y();
            elem.setX(elem.z());
            elem.setY(elem.w());
            elem.setZ(tempX);
            elem.setW(tempY);
        }

        qDebug() << "curve to surface u: ";
        for (const auto& ans : tmp)
        {
            qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
        }
        answer += tmp;
    }

    auto tmp = curveToSurfaceIntersectionU(spline2, spline2->knotU.last(), spline1);
    for (auto& elem : tmp)
    {
        float tempX = elem.x();
        float tempY = elem.y();
        elem.setX(elem.z());
        elem.setY(elem.w());
        elem.setZ(tempX);
        elem.setW(tempY);

        qDebug() << "curve to surface u: ";
        for (const auto& ans : tmp)
        {
            qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
        }
    }
    answer += tmp;

    for (auto V = spline2->knotV.first(); V < spline2->knotV.last(); V += dv2)
    {
        auto tmp = curveToSurfaceIntersectionV(spline2, V, spline1);

        for (auto& elem : tmp)
        {
            float tempX = elem.x();
            float tempY = elem.y();
            elem.setX(elem.z());
            elem.setY(elem.w());
            elem.setZ(tempX);
            elem.setW(tempY);
        }
        qDebug() << "curve to surface v: ";
        for (const auto& ans : tmp)
        {
            qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
        }

        answer += tmp;
    }

    tmp = curveToSurfaceIntersectionV(spline2, spline2->knotV.last(), spline1);

    for (auto& elem : tmp)
    {
        float tempX = elem.x();
        float tempY = elem.y();
        elem.setX(elem.z());
        elem.setY(elem.w());
        elem.setZ(tempX);
        elem.setW(tempY);
    }
    qDebug() << "curve to surface v: ";
    for (const auto& ans : tmp)
    {
        qDebug() << ans << getPoint(spline1, ans[0], ans[1]);
    }
    answer += tmp;

    return answer;
}

QVector<QVector3D> SplineUtils::iterPointsPlus(NURBS *spline1, NURBS *spline2, const QVector4D &point) {
    QVector<QVector3D> result;

    auto initialPoint = point;

    int iterCount = 0;


    while (isBound(spline1, spline2, initialPoint) && iterCount < 1000)
    {

        auto poi = getPoint(spline1, initialPoint.x(), initialPoint.y());
        auto cross = QVector3D::crossProduct(getNormal(spline1, initialPoint.x(), initialPoint.y()),
                                             getNormal(spline2, initialPoint.z(), initialPoint.w())).length();
        qDebug() << "iteration: " << iterCount << " with point " << initialPoint << " and cross: " << cross << " value: " << poi;
        if ( cross < 0.01) {
            qDebug() << "Found tangent intersection in point " << initialPoint;
            return result;
        }
        if (!isBound(spline1, spline2, initialPoint))
        {
            break;
        }

        if (iterCount != 0)
        {

            result.append(getPoint(spline1, initialPoint.x(), initialPoint.y()));
        }

        auto derUV = getDerivatives(spline1, initialPoint.x(), initialPoint.y());
        auto normal1 = QVector3D::crossProduct(derUV.first, derUV.second).normalized();
        auto derAB = getDerivatives(spline2, initialPoint.z(), initialPoint.w());
        auto normal2 = QVector3D::crossProduct(derAB.first, derAB.second).normalized();
        auto t = QVector3D::crossProduct(normal1, normal2);

        if (t.length() < 0.01)
        {
            break;
        }
        auto ur1 = getAdaptiveStepU(spline1, initialPoint.x(), initialPoint.y()) * derUV.first;
        auto vr2 = getAdaptiveStepV(spline1, initialPoint.x(), initialPoint.y()) * derUV.second;
        auto as1 = getAdaptiveStepU(spline2, initialPoint.z(), initialPoint.w()) * derAB.first;
        auto bs2 = getAdaptiveStepV(spline2, initialPoint.z(), initialPoint.w()) * derAB.second;

        auto app = getStepRadius(ur1, vr2, as1, bs2, t);
        auto direction = getDirectionStepRadius(ur1 / derUV.first.length(), vr2 / derUV.second.length(), as1 / derAB.first.length(), bs2 / derAB.second.length(), t);

        auto stepu0 = app * QVector3D::dotProduct(t, derUV.first) / (t.length() * derUV.first.lengthSquared());
        auto stepv0 = app * QVector3D::dotProduct(t, derUV.second) / (t.length() * derUV.second.lengthSquared());
        auto stepa0 = app * QVector3D::dotProduct(t, derAB.first) / (t.length() * derAB.first.lengthSquared());
        auto stepb0 = app * QVector3D::dotProduct(t, derAB.second) / (t.length() * derAB.second.lengthSquared());

        auto u0 = initialPoint.x() + stepu0;
        auto v0 = initialPoint.y() + stepv0;
        auto a0 = initialPoint.z() + stepa0;
        auto b0 = initialPoint.w() + stepb0;

        initialPoint.setX(u0);
        initialPoint.setY(v0);
        initialPoint.setZ(a0);
        initialPoint.setW(b0);

        try
        {
            switch (direction)
            {
                case 0:
                {
                    auto tmp = QVector3D(initialPoint.y(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution3D(spline1, spline2, tmp, 0, initialPoint.x());
                    initialPoint.setY(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 1:
                {
                    auto tmp = QVector3D(initialPoint.x(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution3D(spline1, spline2, tmp, 1, initialPoint.y());
                    initialPoint.setX(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 2:
                {
                    auto tmp = QVector3D(initialPoint.w(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution3D(spline2, spline1, tmp, 0, initialPoint.z());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setW(next.x());
                    break;
                }

                case 3:
                {
                    auto tmp = QVector3D(initialPoint.z(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution3D(spline2, spline1, tmp, 1, initialPoint.w());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setZ(next.x());
                    break;
                }
            }
        } catch (NotBoundException& e)
        {
            qDebug() << e.what();
            return result;
        }


        ++iterCount;
    }

    return result;
}

QVector<QVector3D> SplineUtils::iterPointsMinus(NURBS *spline1, NURBS *spline2, const QVector4D &point) {
    QVector<QVector3D> result;

    auto initialPoint = point;

    int iterCount = 0;


    while (isBound(spline1, spline2, initialPoint) && iterCount < 1000)
    {

        auto poi = getPoint(spline1, initialPoint.x(), initialPoint.y());
        auto cross = QVector3D::crossProduct(getNormal(spline1, initialPoint.x(), initialPoint.y()),
                                             getNormal(spline2, initialPoint.z(), initialPoint.w())).length();
        qDebug() << "iteration: " << iterCount << " with point " << initialPoint << " and cross: " << cross << " value: " << poi;
        if ( cross < 0.01) {
            qDebug() << "Found tangent intersection in point " << initialPoint;
            return result;
        }
        if (!isBound(spline1, spline2, initialPoint))
        {
            break;
        }

        if (iterCount != 0)
        {

            result.append(getPoint(spline1, initialPoint.x(), initialPoint.y()));
        }

        auto derUV = getDerivatives(spline1, initialPoint.x(), initialPoint.y());
        auto normal1 = QVector3D::crossProduct(derUV.first, derUV.second).normalized();
        auto derAB = getDerivatives(spline2, initialPoint.z(), initialPoint.w());
        auto normal2 = QVector3D::crossProduct(derAB.first, derAB.second).normalized();
        auto t = -QVector3D::crossProduct(normal1, normal2);

        if (t.length() < 0.01)
        {
            break;
        }
        auto ur1 = getAdaptiveStepU(spline1, initialPoint.x(), initialPoint.y()) * derUV.first;
        auto vr2 = getAdaptiveStepV(spline1, initialPoint.x(), initialPoint.y()) * derUV.second;
        auto as1 = getAdaptiveStepU(spline2, initialPoint.z(), initialPoint.w()) * derAB.first;
        auto bs2 = getAdaptiveStepV(spline2, initialPoint.z(), initialPoint.w()) * derAB.second;

        auto app = getStepRadius(ur1, vr2, as1, bs2, t);
        auto direction = getDirectionStepRadius(ur1 / derUV.first.length(), vr2 / derUV.second.length(), as1 / derAB.first.length(), bs2 / derAB.second.length(), t);

        auto stepu0 = app * QVector3D::dotProduct(t, derUV.first) / (t.length() * derUV.first.lengthSquared());
        auto stepv0 = app * QVector3D::dotProduct(t, derUV.second) / (t.length() * derUV.second.lengthSquared());
        auto stepa0 = app * QVector3D::dotProduct(t, derAB.first) / (t.length() * derAB.first.lengthSquared());
        auto stepb0 = app * QVector3D::dotProduct(t, derAB.second) / (t.length() * derAB.second.lengthSquared());

        auto u0 = initialPoint.x() + stepu0;
        auto v0 = initialPoint.y() + stepv0;
        auto a0 = initialPoint.z() + stepa0;
        auto b0 = initialPoint.w() + stepb0;

        initialPoint.setX(u0);
        initialPoint.setY(v0);
        initialPoint.setZ(a0);
        initialPoint.setW(b0);

        try
        {
            switch (direction)
            {
                case 0:
                {
                    auto tmp = QVector3D(initialPoint.y(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution3D(spline1, spline2, tmp, 0, initialPoint.x());
                    initialPoint.setY(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 1:
                {
                    auto tmp = QVector3D(initialPoint.x(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution3D(spline1, spline2, tmp, 1, initialPoint.y());
                    initialPoint.setX(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 2:
                {
                    auto tmp = QVector3D(initialPoint.w(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution3D(spline2, spline1, tmp, 0, initialPoint.z());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setW(next.x());
                    break;
                }

                case 3:
                {
                    auto tmp = QVector3D(initialPoint.z(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution3D(spline2, spline1, tmp, 1, initialPoint.w());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setZ(next.x());
                    break;
                }
            }
        } catch (NotBoundException& e)
        {
            qDebug() << e.what();
            return result;
        }


        ++iterCount;
    }

    return result;
}

QMatrix3x3 SplineUtils::getJacobian3D(NURBS *spline1, NURBS *spline2, int constIndex, GLfloat constValue,
                                      const QVector3D &current) {
    QMatrix3x3 answer;

    if (constIndex == 0)
    {
        if (!isBound(spline1, spline2, QVector4D(constValue, current.x(), current.y(), current.z())))
        {
            throw NotBoundException();
        }
        auto der1 = getDerivatives(spline1, constValue, current.x());
        auto der2 = getDerivatives(spline2, current.y(), current.z());
        answer(0, 0) = der1.second.x();
        answer(1, 0) = der1.second.y();
        answer(2, 0) = der1.second.z();

        answer(0, 1) = -der2.first.x();
        answer(1, 1) = -der2.first.y();
        answer(2, 1) = -der2.first.z();

        answer(0, 2) = -der2.second.x();
        answer(1, 2) = -der2.second.y();
        answer(2, 2) = -der2.second.z();
    } else
    {
        if (!isBound(spline1, spline2, QVector4D(current.x(), constValue, current.y(), current.z())))
        {
            throw NotBoundException();
        }
        auto der1 = getDerivatives(spline1, current.x(), constValue);
        auto der2 = getDerivatives(spline2, current.y(), current.z());
        answer(0, 0) = der1.first.x();
        answer(1, 0) = der1.first.y();
        answer(2, 0) = der1.first.z();

        answer(0, 1) = -der2.first.x();
        answer(1, 1) = -der2.first.y();
        answer(2, 1) = -der2.first.z();

        answer(0, 2) = -der2.second.x();
        answer(1, 2) = -der2.second.y();
        answer(2, 2) = -der2.second.z();
    }


    return answer;
}

QMatrix4x4 SplineUtils::getJacobian4D(NURBS *spline1, NURBS *spline2, const QVector4D &current) {
    QMatrix4x4 answer;

    if (!isBound(spline1, spline2, current))
    {
        throw NotBoundException();
    }
    auto r = getPoint(spline1, current.x(), current.y());
    auto s = getPoint(spline2, current.z(), current.w());

    auto dr = getDerivatives(spline1, current.x(), current.y());
    auto ds = getDerivatives(spline2, current.z(), current.w());

    auto d2r = getDerivatives2(spline1, current.x(), current.y());
    auto d2s = getDerivatives2(spline2, current.z(), current.w());

    auto d2r_mixed = getDerivativesMixed(spline1, current.x(), current.y());
    auto d2s_mixed = getDerivativesMixed(spline2, current.z(), current.w());

    answer(0, 0) = QVector3D::dotProduct(dr.first, dr.first) + QVector3D::dotProduct(r - s, d2r.first);
    answer(1, 0) = QVector3D::dotProduct(dr.first, dr.second) + QVector3D::dotProduct(r - s, d2r_mixed);
    answer(2, 0) = -QVector3D::dotProduct(ds.first, dr.first);
    answer(3, 0) = -QVector3D::dotProduct(ds.second, dr.first);

    answer(0, 1) = QVector3D::dotProduct(dr.first, dr.second) + QVector3D::dotProduct(r - s, d2r_mixed);
    answer(1, 1) = QVector3D::dotProduct(dr.second, dr.second) + QVector3D::dotProduct(r - s, d2r.second);
    answer(2, 1) = -QVector3D::dotProduct(ds.first, dr.second);
    answer(3, 1) = -QVector3D::dotProduct(ds.second, dr.second);

    answer(0, 2) = -QVector3D::dotProduct(ds.first, dr.first);
    answer(1, 2) = -QVector3D::dotProduct(ds.first, dr.second);
    answer(2, 2) = QVector3D::dotProduct(ds.first, ds.first) + QVector3D::dotProduct(s - r, d2s.first);
    answer(3, 2) = QVector3D::dotProduct(ds.first, ds.second) + QVector3D::dotProduct(s - r, d2s_mixed);

    answer(0, 3) = -QVector3D::dotProduct(ds.second, dr.first);
    answer(1, 3) = -QVector3D::dotProduct(ds.second, dr.second);
    answer(2, 3) = QVector3D::dotProduct(ds.first, ds.second) + QVector3D::dotProduct(s - r, d2s_mixed);
    answer(3, 3) = QVector3D::dotProduct(ds.second, ds.second) + QVector3D::dotProduct(s - r, d2s.second);
    return answer;

}

QVector3D SplineUtils::NewtonSolution3D(NURBS *spline1, NURBS *spline2, const QVector3D &zeroSolution, int constIndex,
                                        GLfloat constValue) {
    float eps = 0.00001;

    auto previous = zeroSolution;
    auto current = zeroSolution;

    auto iterCount = 0;

    do
    {
        auto jacobian = invertMatrix3D(getJacobian3D(spline1, spline2, constIndex, constValue, current));
        QVector3D value;
        if (constIndex == 0)
        {
            value = getPoint(spline1, constValue, current.x()) - getPoint(spline2, current.y(), current.z()) ;
        } else
        {
            value = getPoint(spline1, current.x(), constValue) - getPoint(spline2, current.y(), current.z());
        }

        previous = current;
        QGenericMatrix<1, 3, float> vec;
        vec(0, 0) = value.x();
        vec(1, 0) = value.y();
        vec(2, 0) = value.z();

        QGenericMatrix<1, 3, float> prev;
        prev(0, 0) = previous.x();
        prev(1, 0) = previous.y();
        prev(2, 0) = previous.z();
        auto tmp = prev - jacobian * vec;
        current = {tmp(0, 0), tmp(1, 0), tmp(2, 0)};
        qDebug() << "Newton accuracy: " << (current - previous).length() << ' ' << current << " iteration: " << iterCount;
        ++iterCount;
    } while ((current - previous).length() > eps && iterCount <= 50);

    if ((current - previous).length() > eps)
    {
        QVector4D solution;
        if (constValue == 0)
        {
            solution.setX(constValue);
            solution.setY(current.x());
            solution.setZ(current.y());
            solution.setW(current.z());

            auto answer = NewtonSolution4D(spline1, spline2, solution);
            QVector3D tmp{answer.y(), answer.z(), answer.w()};
            return tmp;
        }
        else
        {
            solution.setX(current.x());
            solution.setY(constValue);
            solution.setZ(current.y());
            solution.setW(current.z());

            auto answer = NewtonSolution4D(spline1, spline2, solution);
            QVector3D tmp{answer.x(), answer.z(), answer.w()};
            return tmp;
        }
    }
    return current;
}

QVector4D SplineUtils::NewtonSolution4D(NURBS *spline1, NURBS *spline2, const QVector4D &zeroSolution) {
    auto previous = zeroSolution;
    auto current = zeroSolution;
    do
    {
        auto jacobian = invertMatrix4D(getJacobian4D(spline1, spline2, current));
        QVector4D value;
        auto r = getPoint(spline1, zeroSolution.x(), zeroSolution.y());
        auto s = getPoint(spline2, zeroSolution.z(), zeroSolution.w());
        auto dr = getDerivatives(spline1, zeroSolution.x(), zeroSolution.y());
        auto ds = getDerivatives(spline2, zeroSolution.z(), zeroSolution.w());

        value.setX(QVector3D::dotProduct(r - s, dr.first));
        value.setY(QVector3D::dotProduct(r - s, dr.second));
        value.setZ(QVector3D::dotProduct(s - r, ds.first));
        value.setW(QVector3D::dotProduct(s - r, ds.second));

        previous = current;
        QGenericMatrix<1, 4, float> vec;

        current = previous - value * jacobian;
        qDebug() << "Newton accuracy: " << (current - previous).length() << ' ' << current;
    } while ((current - previous).length() > 0.00001);

    return current;
}

bool SplineUtils::isBound(NURBS *spline1, NURBS *spline2, const QVector4D &point) {
    bool inU = point.x() <= spline1->knotU.last() && point.x() >= spline1->knotU.first();
    bool inV = point.y() <= spline1->knotV.last() && point.y() >= spline1->knotV.first();
    bool inA = point.z() <= spline2->knotU.last() && point.z() >= spline2->knotU.first();
    bool inB = point.w() <= spline2->knotV.last() && point.w() >= spline2->knotV.first();

    return inU && inV && inA && inB;
}
