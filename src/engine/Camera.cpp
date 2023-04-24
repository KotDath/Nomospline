#include "engine/Camera.h"

QMatrix4x4 Camera::getView()
{
    view.setToIdentity();
    view.lookAt(eye, center, up);
    return view;
}

void Camera::setPosition(float x, float y, float z)
{
    eye = QVector3D(x, y, z);
}

void Camera::setViewPoint(float x, float y, float z)
{
    center = QVector3D(x, y, z);
}

void Camera::setUpVector(float x, float y, float z)
{
    up = QVector3D(x, y, z);
}

void Camera::setPerpective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane)
{
    projection.setToIdentity();
    projection.perspective(verticalAngle, aspectRatio, nearPlane, farPlane);
}

QMatrix4x4 Camera::getProjection()
{
    return projection;
}
