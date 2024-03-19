#pragma once

#include <QMatrix4x4>
#include <QQuickWindow>

class Camera
{
public:
    Camera();
    Camera(const Camera& camera);
    void setPosition(float x, float y, float z);
    void moveTo(float x, float y, float z);
    QVector3D getPosition();
    void rotateCameraAroundY(float angle);
    void rotateCameraAroundX(float angle);

    void zoomIn(float scale);
    void zoomOut(float scale);

    void setViewPoint(float x, float y, float z);

    void setUpVector(float x, float y, float z);

    void interp(float alpha, QQuickWindow* window);
    void interp(float alpha, QWidget* window);


    QMatrix4x4 getView();
    QMatrix4x4 getProjection();
    void setPerpective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane);

private:

    QMatrix4x4 projection;
    QVector3D eye;
    QVector3D target_eye;
    QVector3D center;
    QVector3D up;

    QMatrix4x4 view;
};