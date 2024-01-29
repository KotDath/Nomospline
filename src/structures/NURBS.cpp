//
// Created by epida on 27.04.2023.
//
#include "structures/NURBS.h"
#include <QSet>

float deltaAlpha = M_PI / 10;
float stepMin = 0.05;
float stepMax = 0.5;

class NotBoundException : public std::exception {
public:
    char * what () {
        return "Point not bound";
    }
};

Mesh *NURBS::evaluate()
{
    Timer timer("Evaluate mesh started", "Evaluate mesh finished");
    auto mesh = new Mesh();
    float N = 100, M = 100;
    float du = (knotU.last() - knotU.first()) / (N - 1), dv = (knotV.last() - knotV.first()) / (M - 1);
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
                auto p1 = getPoint(u - du, v);
                auto p2 = getPoint(u, v);
                auto p3 = getPoint(u + du, v);
                qDebug() << "Points: " << p1 << p2 << p3;
                qDebug() << "Accurate derivative: " << u << v << getDerivatives(u, v).first << "Approximation: "
                         << (p3 - p1) / (du * 2);
                qDebug() << "Accurate 2 derivative: " << u << v << getDerivatives2(u, v).first << "Approximation: "
                         << (p3 - 2 * p2 + p1) / (du * du);
                ++k;
            }


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

    int span_du = span_u;
    int span_dv = span_v;

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
    return QVector3D::crossProduct(pair.first, pair.second).normalized();
}

std::pair<QVector3D, QVector3D> NURBS::getDerivatives2(GLfloat u, GLfloat v)
{
    int span_u = findSpan(uDegree, knotU, u);
    int span_v = findSpan(vDegree, knotV, v);

    auto Nu = basicFunctions(uDegree, span_u, knotU, u);
    auto Nv = basicFunctions(vDegree, span_v, knotV, v);

    int span_du = span_u;
    int span_dv = span_v;

    auto Ndu = basicFunctions(uDegree - 1, span_du, knotU, u);
    auto Ndv = basicFunctions(vDegree - 1, span_dv, knotV, v);

    int span_ddu = span_u;
    int span_ddv = span_v;

    std::vector<float> Nddu, Nddv;

    QVector3D der_v(0, 0, 0);
    QVector3D der_u(0, 0, 0);


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


    if (uDegree >= 2) {
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

        Nddu = basicFunctions(uDegree - 2, span_ddu, knotU, u);

        for (int j = 0; j <= vDegree; j++)
        {
            QVector4D temp;
            for (int i = 0; i < uDegree - 1; i++)
            {
                auto tmp1 = controlPoints[span_u - uDegree + i + 2][span_v - vDegree + j];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = controlPoints[span_u - uDegree + i + 1][span_v - vDegree + j];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto tmp3 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
                tmp3.setX(tmp3.x() * tmp3.w());
                tmp3.setY(tmp3.y() * tmp3.w());
                tmp3.setZ(tmp3.z() * tmp3.w());

                auto pi = uDegree * (tmp1 - tmp2) / (knotU[span_u + i + 2] - knotU[span_u - uDegree + i + 2]);
                auto pi_1 = uDegree * (tmp2 - tmp3) / (knotU[span_u + i + 1] - knotU[span_u - uDegree + i + 1]);

                auto current = (uDegree - 1) * (pi - pi_1) /
                               (knotU[span_u + i + 1] - knotU[span_u - uDegree + i + 2]);
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

    if (vDegree >= 2) {

        for (int i = 0; i <= uDegree; i++) {
            QVector4D temp;
            for (int j = 0; j < vDegree; j++) {
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

        QVector3D dA(point.x(), point.y(), point.z());
        float dB = point.w();

        point.setX(0);
        point.setY(0);
        point.setZ(0);
        point.setW(0);

        Nddv = basicFunctions(vDegree - 2, span_ddv, knotV, v);

        for (int i = 0; i <= uDegree; i++) {
            QVector4D temp;
            for (int j = 0; j < vDegree - 1; j++) {
                auto tmp1 = controlPoints[span_u - uDegree + i][span_v - vDegree + j + 2];
                tmp1.setX(tmp1.x() * tmp1.w());
                tmp1.setY(tmp1.y() * tmp1.w());
                tmp1.setZ(tmp1.z() * tmp1.w());

                auto tmp2 = controlPoints[span_u - uDegree + i][span_v - vDegree + j + 1];
                tmp2.setX(tmp2.x() * tmp2.w());
                tmp2.setY(tmp2.y() * tmp2.w());
                tmp2.setZ(tmp2.z() * tmp2.w());

                auto tmp3 = controlPoints[span_u - uDegree + i][span_v - vDegree + j];
                tmp3.setX(tmp3.x() * tmp3.w());
                tmp3.setY(tmp3.y() * tmp3.w());
                tmp3.setZ(tmp3.z() * tmp3.w());

                auto pi = vDegree * (tmp1 - tmp2) / (knotV[span_v + j + 2] - knotV[span_v - vDegree + j + 2]);
                auto pi_1 = vDegree * (tmp2 - tmp3) / (knotV[span_v + j + 1] - knotV[span_v - vDegree + j + 1]);

                auto current = (vDegree - 1) * (pi - pi_1) /
                               (knotV[span_v + j + 1] - knotV[span_v - vDegree + j + 2]);

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

GLfloat
NURBS::getStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                     const QVector3D &t)
{

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

GLfloat NURBS::getAdaptiveStepU(GLfloat u, GLfloat v)
{

    auto der1 = getDerivatives(u, v);
    auto der2 = getDerivatives2(u, v);
    auto normal = getNormal(u, v);

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

GLfloat NURBS::getAdaptiveStepV(GLfloat u, GLfloat v)
{


    auto der1 = getDerivatives(u, v);
    auto der2 = getDerivatives2(u, v);
    auto normal = getNormal(u, v);

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


QVector<QVector4D> NURBS::getInitialPoints(NURBS *otherSpline)
{
    QVector<QVector4D> answer;

    float stepU = 2;
    float stepV = 2;

    auto du1 = (knotU.last() - knotU.first()) / stepU;
    auto dv1 = (knotV.last() - knotV.first()) / stepV;
    auto du2 = (otherSpline->knotU.last() - otherSpline->knotU.first()) / stepU;
    auto dv2 = (otherSpline->knotV.last() - otherSpline->knotV.first()) / stepV;

    for (auto U = knotU.first(); U < knotU.last(); U += du1)
    {
        answer += curveToSurfaceIntersectionU(U, otherSpline);

        qDebug() << "curve to surface u: ";
        for (const auto& ans : answer)
        {
            qDebug() << ans << this->getPoint(ans[0], ans[1]);
        }
    }

    answer += curveToSurfaceIntersectionU(knotU.last(), otherSpline);

    qDebug() << "curve to surface u: ";
    for (const auto& ans : answer)
    {
        qDebug() << ans << this->getPoint(ans[0], ans[1]);
    }

    for (auto V = knotV.first(); V <= knotV.last(); V += dv1)
    {
        answer += curveToSurfaceIntersectionV(V, otherSpline);

        qDebug() << "curve to surface v: ";
        for (const auto& ans : answer)
        {
            qDebug() << ans << this->getPoint(ans[0], ans[1]);
        }
    }

    answer += curveToSurfaceIntersectionV(knotV.last(), otherSpline);

    qDebug() << "curve to surface v: ";
    for (const auto& ans : answer)
    {
        qDebug() << ans << this->getPoint(ans[0], ans[1]);
    }

    for (auto U = otherSpline->knotU.first(); U < otherSpline->knotU.last(); U += du2)
    {

        auto tmp = otherSpline->curveToSurfaceIntersectionU(U, this);
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
            qDebug() << ans << this->getPoint(ans[0], ans[1]);
        }
        answer += tmp;
    }

    auto tmp = otherSpline->curveToSurfaceIntersectionU(otherSpline->knotU.last(), this);
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
            qDebug() << ans << this->getPoint(ans[0], ans[1]);
        }
    }
    answer += tmp;

    for (auto V = otherSpline->knotV.first(); V < otherSpline->knotV.last(); V += dv2)
    {
        auto tmp = otherSpline->curveToSurfaceIntersectionV(V, this);

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
            qDebug() << ans << this->getPoint(ans[0], ans[1]);
        }

        answer += tmp;
    }

    tmp = otherSpline->curveToSurfaceIntersectionV(otherSpline->knotV.last(), this);

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
        qDebug() << ans << this->getPoint(ans[0], ans[1]);
    }
    answer += tmp;

    return answer;
}

QVector<QVector4D> NURBS::curveToSurfaceIntersectionU(GLfloat u, NURBS *otherSpline)
{

    QVector<QVector4D> intersections;

    GLfloat tempV = knotV.first();

    QMatrix2x2 g;
    QMatrix2x2 tmpMatrix;

    while (tempV < knotV.last())
    {
        auto deltaV = getAdaptiveStepCurveV(u, tempV);
        GLfloat tempA = otherSpline->knotU.first();
        GLfloat tempB = otherSpline->knotV.first();

        auto c0 = getPoint(u, tempV);
        auto c = getDerivatives(u, tempV).second;

        while (tempA < otherSpline->knotU.last())
        {
            auto deltaA = otherSpline->getAdaptiveStepU(tempA, tempB);

            while (tempB < otherSpline->knotV.last())
            {
                auto deltaB = otherSpline->getAdaptiveStepV(tempA, tempB);

                auto der1 = otherSpline->getDerivatives(tempA, tempB); // s1, s2
                auto normal = QVector3D::crossProduct(der1.first, der1.second).normalized();
                auto s0 = otherSpline->getPoint(tempA, tempB);

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
                        auto solution = NewtonSolution(this, otherSpline, zeroSolution, 0, u);
                        QVector4D insertSolution{u, solution.x(), solution.y(), solution.z()};

                        if (isBound(otherSpline, insertSolution)) {
                            intersections.append(insertSolution);
                        }
                    }catch (NotBoundException& e)
                    {
                        qDebug() << e.what();
                    }


                }

                tempB += deltaB;
            }

            tempB = otherSpline->knotV.first();
            tempA += deltaA;
        }

        tempB = otherSpline->knotV.first();
        tempA = otherSpline->knotU.first();
        tempV += deltaV;

    }


    return intersections;
}

QVector<QVector4D> NURBS::curveToSurfaceIntersectionV(GLfloat v, NURBS *otherSpline)
{
    QVector<QVector4D> intersections;

    GLfloat tempU = knotU.first();

    QMatrix2x2 g;
    QMatrix2x2 tmpMatrix;

    while (tempU < knotU.last())
    {
        auto deltaU = getAdaptiveStepCurveU(tempU, v);
        GLfloat tempA = otherSpline->knotU.first();
        GLfloat tempB = otherSpline->knotV.first();

        auto c0 = getPoint(tempU, v);
        auto c = getDerivatives(tempU, v).first;

        while (tempA < otherSpline->knotU.last())
        {
            auto deltaA = otherSpline->getAdaptiveStepU(tempA, tempB);

            while (tempB < otherSpline->knotV.last())
            {
                auto deltaB = otherSpline->getAdaptiveStepV(tempA, tempB);

                auto der1 = otherSpline->getDerivatives(tempA, tempB); // s1, s2
                auto normal = QVector3D::crossProduct(der1.first, der1.second).normalized();
                auto s0 = otherSpline->getPoint(tempA, tempB);

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
                        auto solution = NewtonSolution(this, otherSpline, zeroSolution, 1, v);
                        QVector4D insertSolution{solution.x(), v, solution.y(), solution.z()};
                        if (isBound(otherSpline, insertSolution))
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

            tempB = otherSpline->knotV.first();
            tempA += deltaA;
        }

        tempB = otherSpline->knotV.first();
        tempA = otherSpline->knotU.first();
        tempU += deltaU;

    }


    return intersections;
}

QMatrix3x3
NURBS::getJacobian(NURBS *spline1, NURBS *spline2, int constIndex, GLfloat constValue, const QVector3D &current)
{

    QMatrix3x3 answer;

    if (constIndex == 0)
    {
        if (!spline1->isBound(spline2, QVector4D(constValue, current.x(), current.y(), current.z())))
        {
            throw NotBoundException();
        }
        auto der1 = spline1->getDerivatives(constValue, current.x());
        auto der2 = spline2->getDerivatives(current.y(), current.z());
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
        if (!spline1->isBound(spline2, QVector4D(current.x(), constValue, current.y(), current.z())))
        {
            throw NotBoundException();
        }
        auto der1 = spline1->getDerivatives(current.x(), constValue);
        auto der2 = spline2->getDerivatives(current.y(), current.z());
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

float determinant(QMatrix3x3 mat)
{
    float det = 0;
    for (int i = 0; i < 3; i++)
    {
        det += mat(0, i) * (mat(1, (i + 1) % 3) * mat(2, (i + 2) % 3) - mat(1, (i + 2) % 3) * mat(2, (i + 1) % 3));
    }
    return det;
}

QMatrix3x3 invertMatrix(const QMatrix3x3 &matrix)
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

QVector3D
NURBS::NewtonSolution(NURBS *spline1, NURBS *spline2, const QVector3D &zeroSolution, int constIndex, GLfloat constValue)
{


    auto previous = zeroSolution;
    auto current = zeroSolution;
    do
    {
        auto jacobian = invertMatrix(getJacobian(spline1, spline2, constIndex, constValue, current));
        QVector3D value;
        if (constIndex == 0)
        {
            value = spline1->getPoint(constValue, current.x()) - spline2->getPoint(current.y(), current.z()) ;
        } else
        {
            value = spline1->getPoint(current.x(), constValue) - spline2->getPoint(current.y(), current.z());
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
        qDebug() << "Newton accuracy: " << (current - previous).length() << ' ' << current;
    } while ((current - previous).length() > 0.001);

    return current;
}

GLfloat NURBS::getAdaptiveStepCurveU(GLfloat u, GLfloat v)
{
    auto der1 = getDerivatives(u, v);
    auto der2 = getDerivatives2(u, v);

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

GLfloat NURBS::getAdaptiveStepCurveV(GLfloat u, GLfloat v)
{
    auto der1 = getDerivatives(u, v);
    auto der2 = getDerivatives2(u, v);

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

QVector<QVector3D> NURBS::iterPointsPlus(NURBS *otherSpline, const QVector4D &point)
{
    QVector<QVector3D> result;

    auto initialPoint = point;

    int iterCount = 0;


    while (isBound(otherSpline, initialPoint))
    {
        qDebug() << "iteration: " << iterCount << " with point " << initialPoint;
        auto poi = getPoint(initialPoint.x(), initialPoint.y());
        if (QVector3D::crossProduct(getNormal(initialPoint.x(), initialPoint.y()),
                                    otherSpline->getNormal(initialPoint.z(), initialPoint.w())).length() < 0.001) {
            qDebug() << "Found tangent intersection in point " << initialPoint;
            return result;
        }
        if (!isBound(otherSpline, initialPoint))
        {
            break;
        }

        if (iterCount != 0)
        {

            result.append(getPoint(initialPoint.x(), initialPoint.y()));
        }

        auto derUV = getDerivatives(initialPoint.x(), initialPoint.y());
        auto normal1 = QVector3D::crossProduct(derUV.first, derUV.second).normalized();
        auto derAB = otherSpline->getDerivatives(initialPoint.z(), initialPoint.w());
        auto normal2 = QVector3D::crossProduct(derAB.first, derAB.second).normalized();
        auto t = QVector3D::crossProduct(normal1, normal2);

        if (t.length() < 0.01)
        {
            break;
        }
        auto ur1 = getAdaptiveStepU(initialPoint.x(), initialPoint.y()) * derUV.first;
        auto vr2 = getAdaptiveStepV(initialPoint.x(), initialPoint.y()) * derUV.second;
        auto as1 = otherSpline->getAdaptiveStepU(initialPoint.z(), initialPoint.w()) * derAB.first;
        auto bs2 = otherSpline->getAdaptiveStepV(initialPoint.z(), initialPoint.w()) * derAB.second;

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
                    auto next = NewtonSolution(this, otherSpline, tmp, 0, initialPoint.x());
                    initialPoint.setY(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 1:
                {
                    auto tmp = QVector3D(initialPoint.x(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution(this, otherSpline, tmp, 1, initialPoint.y());
                    initialPoint.setX(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 2:
                {
                    auto tmp = QVector3D(initialPoint.w(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution(otherSpline, this, tmp, 0, initialPoint.z());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setW(next.x());
                    break;
                }

                case 3:
                {
                    auto tmp = QVector3D(initialPoint.z(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution(otherSpline, this, tmp, 1, initialPoint.w());
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

QVector<QVector3D> NURBS::iterPointsMinus(NURBS *otherSpline, const QVector4D &point)
{
    QVector<QVector3D> result;

    auto initialPoint = point;

    int iterCount = 0;


    while (isBound(otherSpline, initialPoint))
    {
        qDebug() << "iteration: " << iterCount << " with point " << initialPoint;
        auto poi = getPoint(initialPoint.x(), initialPoint.y());
        if (QVector3D::crossProduct(getNormal(initialPoint.x(), initialPoint.y()),
                                    otherSpline->getNormal(initialPoint.z(), initialPoint.w())).length() < 0.001) {
            qDebug() << "Found tangent intersection in point " << initialPoint;
            return result;
        }

        if (!isBound(otherSpline, initialPoint))
        {
            break;
        }

        if (iterCount != 0)
        {

            result.append(getPoint(initialPoint.x(), initialPoint.y()));
        }

        auto derUV = getDerivatives(initialPoint.x(), initialPoint.y());
        auto normal1 = QVector3D::crossProduct(derUV.first, derUV.second).normalized();
        auto derAB = otherSpline->getDerivatives(initialPoint.z(), initialPoint.w());
        auto normal2 = QVector3D::crossProduct(derAB.first, derAB.second).normalized();
        auto t = -QVector3D::crossProduct(normal1, normal2);

        auto ur1 = getAdaptiveStepU(initialPoint.x(), initialPoint.y()) * derUV.first;
        auto vr2 = getAdaptiveStepV(initialPoint.x(), initialPoint.y()) * derUV.second;
        auto as1 = otherSpline->getAdaptiveStepU(initialPoint.z(), initialPoint.w()) * derAB.first;
        auto bs2 = otherSpline->getAdaptiveStepV(initialPoint.z(), initialPoint.w()) * derAB.second;

        auto app = getStepRadius(ur1, vr2, as1, bs2, t);
        auto direction = getDirectionStepRadius(ur1 / derUV.first.length(), vr2 / derUV.second.length(), as1 / derAB.first.length(), bs2 / derAB.second.length(), t);

        auto u0 = initialPoint.x() +
                  app * QVector3D::dotProduct(t, derUV.first) / (t.length() * derUV.first.lengthSquared());
        auto v0 = initialPoint.y() + app * QVector3D::dotProduct(t, derUV.second) /
                                     (t.length() * derUV.second.lengthSquared());
        auto a0 = initialPoint.z() +
                  app * QVector3D::dotProduct(t, derAB.first) / (t.length() * derAB.first.lengthSquared());
        auto b0 = initialPoint.w() + app * QVector3D::dotProduct(t, derAB.second) /
                                     (t.length() * derAB.second.lengthSquared());

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
                    auto next = NewtonSolution(this, otherSpline, tmp, 0, initialPoint.x());
                    initialPoint.setY(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 1:
                {
                    auto tmp = QVector3D(initialPoint.x(), initialPoint.z(), initialPoint.w());
                    auto next = NewtonSolution(this, otherSpline, tmp, 1, initialPoint.y());
                    initialPoint.setX(next.x());
                    initialPoint.setZ(next.y());
                    initialPoint.setW(next.z());
                    break;
                }

                case 2:
                {
                    auto tmp = QVector3D(initialPoint.w(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution(otherSpline, this, tmp, 0, initialPoint.z());
                    initialPoint.setX(next.y());
                    initialPoint.setY(next.z());
                    initialPoint.setW(next.x());
                    break;
                }

                case 3:
                {
                    auto tmp = QVector3D(initialPoint.z(), initialPoint.x(), initialPoint.y());
                    auto next = NewtonSolution(otherSpline, this, tmp, 1, initialPoint.w());
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

int
NURBS::getDirectionStepRadius(const QVector3D &ur1, const QVector3D &vr2, const QVector3D &as1, const QVector3D &bs2,
                              const QVector3D &t)
{
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

bool NURBS::isBound(NURBS *otherSpline, const QVector4D& point) {
    bool inU = point.x() <= knotU.last() && point.x() >= knotU.first();
    bool inV = point.y() <= knotV.last() && point.y() >= knotV.first();
    bool inA = point.z() <= otherSpline->knotU.last() && point.z() >= otherSpline->knotU.first();
    bool inB = point.w() <= otherSpline->knotV.last() && point.w() >= otherSpline->knotV.first();

    return inU && inV && inA && inB;
}
