/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "camera.h"

Camera::Camera()
    : Object3D()
{
    // Set default values
    m_fieldOfView = 45.0f;
    m_sceneRadius = 1.0f;
    m_nearPlane = 0.1f;
    m_farPlane = 10.0f;
    m_width = 1;
    m_height = 1;

    m_updateProjectionMatrix();
}

const Matrix4<double>& Camera::getProjectionMatrix() const {
    return m_projectionMatrix;
}

void Camera::setDimensions(uint32_t width, uint32_t height) {
    m_width = width;
    m_height = height;
    m_updateProjectionMatrix();
}

void Camera::setSceneRadius(float radius) {
    m_sceneRadius = radius;
    setClippingPlanes(0.01f * radius, 10.0f * radius);
}

void Camera::setClippingPlanes(float nearDistance, float farDistance) {
    m_nearPlane = nearDistance;
    m_farPlane = farDistance;
    m_updateProjectionMatrix();
}

void Camera::setFieldOfView(float fov) {
    m_fieldOfView = fov;
    m_updateProjectionMatrix();
}

void Camera::setZoom(double fraction) {
    Vector3<double> zoomVector(0, 0, m_sceneRadius * fraction);
    translateLocal(zoomVector);
}

void Camera::translateCamera(float dx, float dy, const Vector3<double>& worldPoint) {
    // Transform the world point into camera coordinates
    Vector3<double> pointCamera = m_transformMatrix.getInverse() * worldPoint;

    // Get the depth
    double z = -pointCamera.z();

    // Find the scaling of dx and dy from windows coordinates to near plane coordinates
    // and from there to camera coordinates at the object's depth
    double aspect = double(m_width) / double(m_height);
    double top = m_nearPlane * std::tan(m_fieldOfView * M_PI / 360.0f);
    double right = top * aspect;

    // Translate the camera
    translateLocal({2.0f * dx * right / m_nearPlane * z,-2.0f * dy * top / m_nearPlane * z,0.0f});
}

uint32_t Camera::getWidth() const {
    return m_width;
}

uint32_t Camera::getHeight() const {
    return m_height;
}

void Camera::m_updateProjectionMatrix() {
    // Compute the aspect ratio
    double aspect = double(m_width) / double(m_height);

    double top = m_nearPlane * std::tan((m_fieldOfView / 2.0f) * (M_PI / 180.0f));
    double bottom = -top;
    double left = bottom * aspect;
    double right = top * aspect;

    double fx = 2.0f * m_nearPlane / (right - left);
    double fy = 2.0f * m_nearPlane / (top - bottom);
    double fz = -(m_farPlane + m_nearPlane) / (m_farPlane - m_nearPlane);
    double fw = -2.0f * m_farPlane * m_nearPlane / (m_farPlane - m_nearPlane);

    // Recompute the projection matrix
    m_projectionMatrix = {{fx, 0, 0, 0},
                          {0, fy, 0, 0},
                          {0, 0, fz, fw},
                          {0, 0, -1, 0}};
}
