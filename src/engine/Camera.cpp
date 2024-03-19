#include "engine/Camera.h"
#include <QWidget>

QMatrix4x4 Camera::getView() {
    view.setToIdentity();
    view.lookAt(eye, center, up);
    return view;
}

void Camera::setPosition(float x, float y, float z) {
    eye = QVector3D(x, y, z);
    target_eye = eye;
}

void Camera::setViewPoint(float x, float y, float z) {
    center = QVector3D(x, y, z);
}

void Camera::setUpVector(float x, float y, float z) {
    up = QVector3D(x, y, z);
}

void Camera::setPerpective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane) {
    projection.setToIdentity();
    projection.perspective(verticalAngle, aspectRatio, nearPlane, farPlane);
}

QMatrix4x4 Camera::getProjection() {
    return projection;
}

void Camera::rotateCameraAroundY(float angle) {
    QMatrix4x4 rotationEye;
    rotationEye.rotate(angle, 0, 1, 0);
    eye = rotationEye.map(eye);
    target_eye = eye;
}

void Camera::rotateCameraAroundX(float angle) {
    QVector3D rotateAxis(eye.z(), 0, -eye.x());
    rotateAxis.normalize();

    QMatrix4x4 rotationEye;
    rotationEye.rotate(angle, rotateAxis);
    eye = rotationEye.map(eye);
    target_eye = eye;
}

void Camera::zoomIn(float scale) {
    target_eye /= scale;
}

void Camera::zoomOut(float scale) {
    target_eye *= scale;
}

void Camera::interp(float alpha, QQuickWindow *window) {
    if (target_eye == eye) {
        return;
    }
    if ((target_eye - eye).length() < 0.001) {
        eye = target_eye;
        if (window) {
            window->update();
        }

        return;
    }
    alpha = qBound(0.0f, alpha, 1.0f);
    eye = eye * alpha + (1 - alpha) * target_eye;
    if (window) {
        window->update();
    }
}

void Camera::interp(float alpha, QWidget *window) {
    if (target_eye == eye) {
        return;
    }
    if ((target_eye - eye).length() < 0.001) {
        eye = target_eye;
        if (window) {
            window->update();
        }

        return;
    }
    alpha = qBound(0.0f, alpha, 1.0f);
    eye = eye * alpha + (1 - alpha) * target_eye;
    if (window) {
        window->update();
    }
}

QVector3D Camera::getPosition() {
    return eye;
}

Camera::Camera(const Camera &camera) : center(camera.center), eye(camera.eye), up(camera.up) {

}

Camera::Camera() : center(QVector3D(0, 0, 0)), eye(QVector3D(0, 0, 0)), up(QVector3D(0, 1, 0)) {

}

void Camera::moveTo(float x, float y, float z) {
    target_eye = QVector3D(x, y, z);
}
