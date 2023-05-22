//
// Created by epida on 27.04.2023.
//
#include "structures/NURBS.h"

Mesh *NURBS::evaluate()
{
    return nullptr;
}

void NURBS::init()
{

}

int NURBS::findSpan(GLsizei degree, const QVector<GLfloat>& knots, GLfloat param)
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
        }
        else
        {
            low = mid;
        }
        mid = static_cast<GLsizei>(std::floor((low + high) / 2.0));
    }
    return mid;
}

std::vector<GLfloat> NURBS::basicFunctions(GLsizei deg, GLsizei span, const QVector<GLfloat>& knots, GLfloat u)
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
