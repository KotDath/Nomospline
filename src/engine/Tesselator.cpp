#include "engine/Tesselator.h"
#include "structures/SplineUtils.h"
#include "engine/SceneData.h"
#include "structures/BREP.h"
#include "logger.h"

struct PointInfo {
    GLfloat u;
    GLfloat v;
    QVector3D coord;
};

QVector<QVector4D> filterPoints(NURBS *spl, const QVector<QVector4D> &points) {
    const float epsilon = 0.001;
    QVector<QVector4D> answer;
    for (const auto &point1: points) {
        if (answer.isEmpty()) {
            answer.append(point1);
        } else {
            float min = (SplineUtils::getPoint(spl, point1.x(), point1.y()) -
                         SplineUtils::getPoint(spl, answer[0].x(), answer[0].y())).length();
            for (const auto &point2: answer) {
                float tmp = (SplineUtils::getPoint(spl, point1.x(), point1.y()) -
                             SplineUtils::getPoint(spl, point2.x(), point2.y())).length();
                min = std::min(min, tmp);
            }

            if (min > epsilon) {
                answer.append(point1);
            }
        }

    }

    return answer;
}

QVector<std::pair<QVector2D, QVector2D>> filter_cluster(const QVector<std::pair<QVector2D, QVector2D>>& row) {
    QVector<std::pair<QVector2D, QVector2D>> answer;
    for (const auto& edge : row) {
        bool isCorrect = true;
        for (const auto& other_edge : row) {
            if (other_edge != edge && edge.first == other_edge.second) {
                isCorrect = false;
            }
        }

        if (isCorrect) {
            answer.append(edge);
        }
    }

    return answer;
}

Mesh *Tesselator::tesselate(NURBS *spl, QVector<QVector<QVector2D>> limits, int offset, bool ignoreTrim) {
    auto mesh = new Mesh();

    auto logger = Logger::getInstance();
    logger->write(spl->knotU.first(), spl->knotU.last(),
                  spl->knotV.first(), spl->knotV.last());

    for (const auto& limit : limits) {
        for (const auto& point : limit) {
            logger->write(point.x(), point.y());
        }
    }

    logger->deployData(QString::number(rand()%(1000+1)));

    // Создание равномерной сетки
    const int N = 65, M = 65;

    if (spl->uDegree == 1 && spl->vDegree == 1) {
        qDebug() << "Limits: " << limits;
    }


    auto clasters = clasterize(spl, limits);


    QVector<QVector<int>> pointTypes(M + 1, QVector<int>(N + 1, 1));


    float du = (spl->knotU.last() - spl->knotU.first()) / N, dv = (spl->knotV.last() -
                                                                   spl->knotV.first()) / M;

    if (!ignoreTrim) {
        for (int j = 0; j <= M; ++j) {
            float v = j * dv;
            if (v == 0.5 && spl->uDegree == 1 && spl->vDegree == 1) {
                qDebug() << "Jopa";
            }
            for (int i = 0; i <= N; ++i) {
                float u = i * du;

                auto crossedLines = filter_cluster(clasters[std::make_pair(u, v)]);
                qDebug() << u << ' ' << v << ' ' << crossedLines;
                for (const auto &edge: crossedLines) {
                    auto P1 = edge.first;
                    auto P2 = edge.second;
                    if (P1.y() > P2.y()) {
                        std::swap(P1, P2);
                    }

                    if (P1.y() != P2.y()) {
                        pointTypes[j][i] += 1;
                    }

                    /*if (crossProduct({u, v}, P1, P2) <= 0.00001f) {
                        pointTypes[j][i] = 1;
                    }*/
                }
            }
        }
        for (int j = 0; j <= M; ++j) {
            for (int i = 0; i <= N; ++i) {
                pointTypes[j][i] = (pointTypes[j][i] + offset - 1) % 2;
            }
        }
    }


    QVector<int> chunks;

    if (spl->uDegree == 1 && spl->vDegree == 2) {
        qDebug() << "Cylinder!";
    }

    float CycledU = (SplineUtils::getPoint(spl, spl->knotU.first(), spl->knotV.first()) -
                     SplineUtils::getPoint(spl, spl->knotU.last(), spl->knotV.first())).length();

    float CycledV = (SplineUtils::getPoint(spl, spl->knotU.first(), spl->knotV.first()) -
                     SplineUtils::getPoint(spl, spl->knotU.first(), spl->knotV.last())).length();


    qDebug() << "Point type: ";

    for (int j = 0; j <= M; ++j) {
        qDebug() << j * dv << pointTypes[j];
    }

    for (int j = 0; j < M; ++j) {
        for (int i = 0; i < N; ++i) {

            if (pointTypes[j][i] != 0 && pointTypes[j][i + 1] != 0 && pointTypes[j + 1][i] != 0 &&
                pointTypes[j + 1][i + 1] != 0) {
                chunks.append(i + j * (N + 1));
                chunks.append(i + (j + 1) * (N + 1));
                chunks.append(i + 1 + (j + 1) * (N + 1));
                chunks.append(i + 1 + j * (N + 1));
            }
        }
    }

    if (CycledU < 0.001f) {
        auto i = N;
        for (int j = 0; j < M; ++j) {
            chunks.append(i + j * (N + 1));
            chunks.append(i + (j + 1) * (N + 1));
            chunks.append(0 + (j + 1) * (N + 1));
            chunks.append(0 + j * (N + 1));
        }
    }


    if (CycledV < 0.001f) {
        qDebug() << "Cycle detected!";
        auto j = M;
        for (int i = 0; i < N; ++i) {
            chunks.append(i + j * (N + 1));
            chunks.append(i + 0 * (N + 1));
            chunks.append(i + 1 + 0 * (N + 1));
            chunks.append(i + 1 + j * (N + 1));
        }
    }

    qDebug() << "Chunks: " << chunks;

    mesh->vertices.reserve((N + 1) * (M + 1));
    for (int j = 0; j <= M; ++j) {
        for (int i = 0; i <= N; ++i) {

            float v = j * dv;


            float u = i * du;
            mesh->vertices.append(VertexData(SplineUtils::getPoint(spl, u, v), SplineUtils::getNormal(spl, u, v)));
        }
    }


    mesh->indices.reserve(chunks.size());


    for (int i = 0; i < chunks.size() / 4; ++i) {

        qDebug() << "Chunk: " << chunks[4 * i] << chunks[4 * i + 1] << chunks[4 * i + 2] << chunks[4 * i + 3];
        qDebug() << "Square with coords: " << mesh->vertices[chunks[4 * i]].position
                 << mesh->vertices[chunks[4 * i + 1]].position << mesh->vertices[chunks[4 * i + 2]].position
                 << mesh->vertices[chunks[4 * i + 3]].position;

        mesh->indices.append(chunks[4 * i]);
        mesh->indices.append(chunks[4 * i + 3]);
        mesh->indices.append(chunks[4 * i + 1]);

        mesh->indices.append(chunks[4 * i + 3]);
        mesh->indices.append(chunks[4 * i + 2]);
        mesh->indices.append(chunks[4 * i + 1]);


    }


    //qDebug() << "indices chunks: " << mesh->indices;
    return mesh;


}

float Tesselator::crossProduct(const QVector2D &p, const QVector2D &P1, const QVector2D &P2) {
    QVector2D l1 = P2 - P1;
    QVector2D l2 = p - P1;

    auto res = l1.x() * l2.y() - l2.x() * l1.y();
    return res;
}

std::map<std::pair<float, float>, QVector<std::pair<QVector2D, QVector2D>>>
Tesselator::clasterize(NURBS *spl, QVector<QVector<QVector2D>> limits) {
    const int N = 65, M = 65;
    std::map<std::pair<float, float>, QVector<std::pair<QVector2D, QVector2D>>> result;

    float du = (spl->knotU.last() - spl->knotU.first()) / N, dv = (spl->knotV.last() -
                                                                   spl->knotV.first()) / M;


    for (const auto &curl: limits) {
        for (int i = 0; i < curl.size() - 1; ++i) {
            auto P0 = curl[i];
            auto P1 = curl[i + 1];


            for (int j = 0; j <= M; ++j) {
                for (int k = 0; k <= N; ++k) {
                    auto v = dv * j;
                    auto u = du * k;

                    if (RayToLineIntersection({u, v}, P0, P1)) {
                        result[std::make_pair(u, v)].append({P0, P1});
                    } else {
                        /*
                        if (v == std::min(P0.y(), P1.y())) {
                            if (v == 0) {
                                if (P0.y() == 0 && u <= P0.x() && P1.y() != P0.y()) {
                                    result[std::make_pair(u, v)].append({P0, P1});
                                }

                                if (P1.y() == 0 && u <= P1.x() && P1.y() != P0.y()) {
                                    result[std::make_pair(u, v)].append({P0, P1});
                                }


                            }
                        }
                         */
                    }
                }
            }
        }
    }

    qDebug() << "Tmp point" << result[std::make_pair(0.5, 1)];

    return result;
}

bool Tesselator::RayToLineIntersection(const QVector2D &sourcePoint, const QVector2D &p1,
                                       const QVector2D &p2) {


    if (sourcePoint.x() > std::max(p1.x(), p2.x())) {
        return false;
    }


    if (sourcePoint.y() < std::min(p1.y(), p2.y())) {
        return false;
    }

    if (sourcePoint.y() > std::max(p1.y(), p2.y())) {
        return false;
    }

    if (p1.x() == p2.x()) {
        return true;
    }

    if (p1.y() == p2.y()) {
        return true;
    }

    auto t = (sourcePoint.y() - p1.y()) / (p2.y() - p1.y());
    auto x_candidate = p1.x() * (1 - t) + t * p2.x();

    return x_candidate >= sourcePoint.x();

}

void Tesselator::intersection(BREP *b1, BREP *b2) {
    auto data = SceneData::getInstance();
    std::map<NURBS *, QVector<NURBS *>> intersectionCount;

    for (const auto &spl1: b1->surfaces.keys()) {
        for (const auto &spl2: b2->surfaces.keys()) {
            if (SplineUtils::isBBIntersected(spl1, spl2)) {
                intersectionCount[spl1].append(spl2);
                intersectionCount[spl2].append(spl1);
            }
        }
    }

    for (const auto &spline: b2->surfaces.keys()) {
        intersectionCount[spline].clear();
    }

    data->intersectionPoints = new Mesh();


    QMap<NURBS *, QVector<QVector<QVector2D>>> trimming;

    for (const auto &pair: intersectionCount) {
        auto spl1 = pair.first;
        for (const auto &spl2: pair.second) {

            QVector<QVector2D> limit_first, limit_second;
            auto intersection = SplineUtils::getInitialPoints(spl1, spl2);
            qDebug() << "intersection: " << intersection;

            intersection = filterPoints(spl1, intersection);
            qDebug() << "intersection after filtering: " << intersection;

            QVector<QVector4D> iterInter;

            for (int i = 0; i < intersection.size(); ++i) {
                auto startPoint = intersection[i];

                if (QVector3D::crossProduct(SplineUtils::getNormal(spl1, startPoint.x(), startPoint.y()),
                                            SplineUtils::getNormal(spl2, startPoint.z(),
                                                                   startPoint.w())).length() < 0.001) {
                    qDebug() << "Found tangent intersection in point " << startPoint;
                }
                Timer timer("Plus side started", "Plus side finished");
                auto iterIntersectionPlus = SplineUtils::iterPointsPlus(spl1, spl2, i, intersection);
                iterInter.append(iterIntersectionPlus);
                for (auto p: iterIntersectionPlus) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(spl1, p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
                timer = Timer("Minus side started", "Minus side finished");
                startPoint = intersection[i];
                auto iterIntersectionMinus = SplineUtils::iterPointsMinus(spl1, spl2, i, intersection);
                for (auto p: iterIntersectionMinus) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(spl1, p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();

                QVector<QVector2D> trim_B1, trim_B2;

                if (iterIntersectionPlus.empty() && iterIntersectionMinus.empty()) {
                    continue;
                }

                if (iterIntersectionPlus.empty()) {
                    auto last_minus = iterIntersectionMinus.last();

                    bool isNearB1 = (startPoint.x() - last_minus.x()) * (startPoint.x() - last_minus.x()) +
                                    (startPoint.y() - last_minus.y()) * (startPoint.y() - last_minus.y()) < 0.01f;

                    bool isNearB2 = (startPoint.z() - last_minus.z()) * (startPoint.z() - last_minus.z()) +
                                    (startPoint.w() - last_minus.w()) * (startPoint.w() - last_minus.w()) < 0.01f;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionMinus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        trim_B1.append({iterIntersectionMinus.last().x(), iterIntersectionMinus.last().y()});
                    } else {
                        trim_B1.reserve(iterIntersectionMinus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});


                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionMinus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        trim_B2.append({iterIntersectionMinus.last().z(), iterIntersectionMinus.last().w()});
                    } else {
                        trim_B2.reserve(iterIntersectionMinus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});

                    }
                } else if (iterIntersectionMinus.empty()) {
                    auto last_plus = iterIntersectionPlus.last();
                    bool isNearB1 = (last_plus.x() - startPoint.x()) * (last_plus.x() - startPoint.x()) +
                                    (last_plus.y() - startPoint.y()) * (last_plus.y() - startPoint.y()) < 0.01f;

                    bool isNearB2 = (last_plus.z() - startPoint.z()) * (last_plus.z() - startPoint.z()) +
                                    (last_plus.w() - startPoint.w()) * (last_plus.w() - startPoint.w()) < 0.01f;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionPlus.size() + 2);
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                    } else {
                        trim_B1.reserve(iterIntersectionPlus.size() + 1);
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionPlus.size() + 2);
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                    } else {
                        trim_B2.reserve(iterIntersectionPlus.size() + 1);
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                    }
                } else {
                    auto last_plus = iterIntersectionPlus.last();
                    auto last_minus = iterIntersectionMinus.last();

                    bool isNearB1 = (last_plus.x() - last_minus.x()) * (last_plus.x() - last_minus.x()) +
                                    (last_plus.y() - last_minus.y()) * (last_plus.y() - last_minus.y()) < 0.01f;

                    bool isNearB2 = (last_plus.z() - last_minus.z()) * (last_plus.z() - last_minus.z()) +
                                    (last_plus.w() - last_minus.w()) * (last_plus.w() - last_minus.w()) < 0.01f;


                    qDebug() << last_plus << last_minus;
                    qDebug() << isNearB1 << isNearB2;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                        trim_B1.append({iterIntersectionMinus.last().x(), iterIntersectionMinus.last().y()});
                    } else {
                        trim_B1.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }


                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                        trim_B2.append({iterIntersectionMinus.last().z(), iterIntersectionMinus.last().w()});
                    } else {
                        trim_B2.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }


                    }
                }

                trim_B1.first() = SplineUtils::fitPointBound(spl1, trim_B1.first());
                trim_B1.last() = SplineUtils::fitPointBound(spl1, trim_B1.last());

                trim_B2.first() = SplineUtils::fitPointBound(spl2, trim_B2.first());
                trim_B2.last() = SplineUtils::fitPointBound(spl2, trim_B2.last());

                qDebug() << "Trimmings: " << trim_B1;
                trimming[spl1].append(trim_B1);
                trimming[spl2].append(trim_B2);

            }

        }
    }

    intersectionCount.clear();

    for (const auto &spl1: b1->surfaces.keys()) {
        for (const auto &spl2: b2->surfaces.keys()) {
            if (SplineUtils::isBBIntersected(spl1, spl2)) {
                intersectionCount[spl1].append(spl2);
                intersectionCount[spl2].append(spl1);
            }
        }
    }

    for (const auto &spline: b1->surfaces.keys()) {
        if (!intersectionCount[spline].empty()) {
            QVector<QVector<QVector2D>> limits;
            for (const auto &curv: b1->surfaces[spline]) {
                limits.append(curv->sequencePoints);
            }


            for (const auto &q: trimming[spline]) {
                limits.append(q);
            }


            if (limits.empty()) {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, true));
            } else {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, false));
            }
        }

    }

    for (const auto &spline: b2->surfaces.keys()) {


        if (!intersectionCount[spline].empty()) {
            QVector<QVector<QVector2D>> limits;
            for (const auto &curv: b2->surfaces[spline]) {
                limits.append(curv->sequencePoints);
            }

            limits.clear();
            for (const auto &q: trimming[spline]) {
                limits.append(q);
            }
            if (limits.empty()) {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, true));
            } else {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, false));
            }
        }
    }

}

void Tesselator::diff(BREP *b1, BREP *b2) {
    auto data = SceneData::getInstance();
    std::map<NURBS *, QVector<NURBS *>> intersectionCount;

    for (const auto &spl1: b1->surfaces.keys()) {
        for (const auto &spl2: b2->surfaces.keys()) {
            if (SplineUtils::isBBIntersected(spl1, spl2)) {
                intersectionCount[spl1].append(spl2);
                intersectionCount[spl2].append(spl1);
            }
        }
    }

    for (const auto &spline: b2->surfaces.keys()) {
        intersectionCount[spline].clear();
    }

    data->intersectionPoints = new Mesh();


    QMap<NURBS *, QVector<QVector<QVector2D>>> trimming;

    for (const auto &pair: intersectionCount) {
        auto spl1 = pair.first;
        for (const auto &spl2: pair.second) {

            QVector<QVector2D> limit_first, limit_second;
            auto intersection = SplineUtils::getInitialPoints(spl1, spl2);
            qDebug() << "intersection: " << intersection;

            intersection = filterPoints(spl1, intersection);
            qDebug() << "intersection after filtering: " << intersection;

            QVector<QVector4D> iterInter;

            for (int i = 0; i < intersection.size(); ++i) {
                auto startPoint = intersection[i];

                if (QVector3D::crossProduct(SplineUtils::getNormal(spl1, startPoint.x(), startPoint.y()),
                                            SplineUtils::getNormal(spl2, startPoint.z(),
                                                                   startPoint.w())).length() < 0.001) {
                    qDebug() << "Found tangent intersection in point " << startPoint;
                }
                Timer timer("Plus side started", "Plus side finished");
                auto iterIntersectionPlus = SplineUtils::iterPointsPlus(spl1, spl2, i, intersection);
                iterInter.append(iterIntersectionPlus);
                for (auto p: iterIntersectionPlus) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(spl1, p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();
                timer = Timer("Minus side started", "Minus side finished");
                startPoint = intersection[i];
                auto iterIntersectionMinus = SplineUtils::iterPointsMinus(spl1, spl2, i, intersection);
                for (auto p: iterIntersectionMinus) {
                    data->intersectionPoints->vertices.append(SplineUtils::getPoint(spl1, p.x(), p.y()));
                    data->intersectionPoints->indices.append(data->intersectionPoints->indices.length());
                }
                timer.Stop();

                QVector<QVector2D> trim_B1, trim_B2;

                if (iterIntersectionPlus.empty() && iterIntersectionMinus.empty()) {
                    continue;
                }

                if (iterIntersectionPlus.empty()) {
                    auto last_minus = iterIntersectionMinus.last();

                    bool isNearB1 = (startPoint.x() - last_minus.x()) * (startPoint.x() - last_minus.x()) +
                                    (startPoint.y() - last_minus.y()) * (startPoint.y() - last_minus.y()) < 0.01f;

                    bool isNearB2 = (startPoint.z() - last_minus.z()) * (startPoint.z() - last_minus.z()) +
                                    (startPoint.w() - last_minus.w()) * (startPoint.w() - last_minus.w()) < 0.01f;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionMinus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        trim_B1.append({iterIntersectionMinus.last().x(), iterIntersectionMinus.last().y()});
                    } else {
                        trim_B1.reserve(iterIntersectionMinus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});


                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionMinus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        trim_B2.append({iterIntersectionMinus.last().z(), iterIntersectionMinus.last().w()});
                    } else {
                        trim_B2.reserve(iterIntersectionMinus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});

                    }
                } else if (iterIntersectionMinus.empty()) {
                    auto last_plus = iterIntersectionPlus.last();
                    bool isNearB1 = (last_plus.x() - startPoint.x()) * (last_plus.x() - startPoint.x()) +
                                    (last_plus.y() - startPoint.y()) * (last_plus.y() - startPoint.y()) < 0.01f;

                    bool isNearB2 = (last_plus.z() - startPoint.z()) * (last_plus.z() - startPoint.z()) +
                                    (last_plus.w() - startPoint.w()) * (last_plus.w() - startPoint.w()) < 0.01f;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionPlus.size() + 2);
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                    } else {
                        trim_B1.reserve(iterIntersectionPlus.size() + 1);
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionPlus.size() + 2);
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                    } else {
                        trim_B2.reserve(iterIntersectionPlus.size() + 1);
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                    }
                } else {
                    auto last_plus = iterIntersectionPlus.last();
                    auto last_minus = iterIntersectionMinus.last();

                    bool isNearB1 = (last_plus.x() - last_minus.x()) * (last_plus.x() - last_minus.x()) +
                                    (last_plus.y() - last_minus.y()) * (last_plus.y() - last_minus.y()) < 0.01f;

                    bool isNearB2 = (last_plus.z() - last_minus.z()) * (last_plus.z() - last_minus.z()) +
                                    (last_plus.w() - last_minus.w()) * (last_plus.w() - last_minus.w()) < 0.01f;


                    qDebug() << last_plus << last_minus;
                    qDebug() << isNearB1 << isNearB2;

                    if (isNearB1) {
                        trim_B1.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }
                        trim_B1.append({iterIntersectionMinus.last().x(), iterIntersectionMinus.last().y()});
                    } else {
                        trim_B1.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B1.append({iterIntersectionMinus[j].x(), iterIntersectionMinus[j].y()});
                        }
                        trim_B1.append({startPoint.x(), startPoint.y()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B1.append({iterIntersectionPlus[j].x(), iterIntersectionPlus[j].y()});
                        }


                    }

                    if (isNearB2) {
                        trim_B2.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 2);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }
                        trim_B2.append({iterIntersectionMinus.last().z(), iterIntersectionMinus.last().w()});
                    } else {
                        trim_B2.reserve(iterIntersectionMinus.size() + iterIntersectionPlus.size() + 1);
                        for (int j = iterIntersectionMinus.size() - 1; j >= 0; --j) {
                            trim_B2.append({iterIntersectionMinus[j].z(), iterIntersectionMinus[j].w()});
                        }
                        trim_B2.append({startPoint.z(), startPoint.w()});
                        for (int j = 0; j < iterIntersectionPlus.size(); ++j) {
                            trim_B2.append({iterIntersectionPlus[j].z(), iterIntersectionPlus[j].w()});
                        }


                    }
                }

                trim_B1.first() = SplineUtils::fitPointBound(spl1, trim_B1.first());
                trim_B1.last() = SplineUtils::fitPointBound(spl1, trim_B1.last());

                trim_B2.first() = SplineUtils::fitPointBound(spl2, trim_B2.first());
                trim_B2.last() = SplineUtils::fitPointBound(spl2, trim_B2.last());

                qDebug() << "Trimmings: " << trim_B1;
                trimming[spl1].append(trim_B1);
                trimming[spl2].append(trim_B2);

            }

        }
    }

    intersectionCount.clear();

    for (const auto &spl1: b1->surfaces.keys()) {
        for (const auto &spl2: b2->surfaces.keys()) {
            if (SplineUtils::isBBIntersected(spl1, spl2)) {
                intersectionCount[spl1].append(spl2);
                intersectionCount[spl2].append(spl1);
            }
        }
    }

    for (const auto &spline: b1->surfaces.keys()) {
        QVector<QVector<QVector2D>> limits;
        for (const auto &curv: b1->surfaces[spline]) {
            limits.append(curv->sequencePoints);
        }


        for (const auto &q: trimming[spline]) {
            limits.append(q);
        }

        int offset = 1;

        if (limits.empty()) {
            data->splineMeshes.append(Tesselator::tesselate(spline, limits, offset, true));
        } else {
            data->splineMeshes.append(Tesselator::tesselate(spline, limits, offset, false));
        }

    }

    for (const auto &spline: b2->surfaces.keys()) {


        if (!intersectionCount[spline].empty()) {
            QVector<QVector<QVector2D>> limits;
            for (const auto &curv: b2->surfaces[spline]) {
                limits.append(curv->sequencePoints);
            }

            limits.clear();
            for (const auto &q: trimming[spline]) {
                limits.append(q);
            }
            if (limits.empty()) {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, true));
            } else {
                data->splineMeshes.append(Tesselator::tesselate(spline, limits, 0, false));
            }
        }
    }
}
