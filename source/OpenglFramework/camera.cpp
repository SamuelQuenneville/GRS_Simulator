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

void Camera::setDimensions(const uint32_t width, const uint32_t height) {
    m_width = width;
    m_height = height;
    m_updateProjectionMatrix();
}

void Camera::setSceneRadius(const float radius) {
    m_sceneRadius = radius;
    setClippingPlanes(0.01f * radius, 10.0f * radius);
}

void Camera::setClippingPlanes(const float nearDistance, const float farDistance) {
    m_nearPlane = nearDistance;
    m_farPlane = farDistance;
    m_updateProjectionMatrix();
}

void Camera::setFieldOfView(const float fov) {
    m_fieldOfView = fov;
    m_updateProjectionMatrix();
}

void Camera::setZoom(const double fraction) {
    const Vector3<double> zoomVector(0.0, 0.0, m_sceneRadius * fraction);
    translateLocal(zoomVector);
}

void Camera::translateCamera(const float dx, const float dy, const Vector3<double>& worldPoint) {
    // Transform the world point into camera coordinates
    const Vector3<double> pointCamera = m_transformMatrix.getInverse() * worldPoint;

    // Get the depth
    const double z = -pointCamera.z();

    // Find the scaling of dx and dy from windows coordinates to near plane coordinates
    // and from there to camera coordinates at the object's depth
    const double aspect = static_cast<double>(m_width) / static_cast<double>(m_height);
    const double top = m_nearPlane * std::tan(m_fieldOfView * M_PI / 360.0);
    const double right = top * aspect;

    // Translate the camera
    translateLocal({2.0 * dx * right / m_nearPlane * z,-2.0 * dy * top / m_nearPlane * z,0.0});
}

uint32_t Camera::getWidth() const {
    return m_width;
}

uint32_t Camera::getHeight() const {
    return m_height;
}

void Camera::m_updateProjectionMatrix() {
    // Compute the aspect ratio
    const double aspect = static_cast<double>(m_width) / static_cast<double>(m_height);

    const double top = m_nearPlane * std::tan(m_fieldOfView  * M_PI / 360.0);
    const double bottom = -top;
    const double left = bottom * aspect;
    const double right = top * aspect;

    double fx = 2.0 * m_nearPlane / (right - left);
    double fy = 2.0 * m_nearPlane / (top - bottom);
    double fz = -(m_farPlane + m_nearPlane) / (m_farPlane - m_nearPlane);
    double fw = -2.0 * m_farPlane * m_nearPlane / (m_farPlane - m_nearPlane);

    // Recompute the projection matrix
    m_projectionMatrix = {{fx, 0, 0, 0},
                          {0, fy, 0, 0},
                          {0, 0, fz, fw},
                          {0, 0, -1, 0}};
}
