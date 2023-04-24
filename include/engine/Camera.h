#pragma once

#include <QMatrix4x4>

class Camera
{
public:
    void setPosition(float x, float y, float z);

    void setViewPoint(float x, float y, float z);

    void setUpVector(float x, float y, float z);


    QMatrix4x4 getView();
    QMatrix4x4 getProjection();
    void setPerpective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane);

private:

    QMatrix4x4 projection;
    QVector3D eye;
    QVector3D center;
    QVector3D up;

    QMatrix4x4 view;
};