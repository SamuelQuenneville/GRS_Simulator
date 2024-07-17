/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "object3D.h"

Object3D::Object3D() {
    setToIdentity();
}

const Matrix4<double>& Object3D::getTransformMatrix() const {
    return m_transformMatrix;
}

void Object3D::setTransformMatrix(const Matrix4<double>& matrix) {
    m_transformMatrix = matrix;
}

void Object3D::setToIdentity() {
    m_transformMatrix.setToIdentity();
}

Vector3<double> Object3D::getOrigin() const {
    return m_transformMatrix * Vector3<double>(0.0, 0.0, 0.0);
}

void Object3D::translateWorld(const Vector3<double>& vector) {
    m_transformMatrix = Matrix4<double>::translationMatrix(vector) * m_transformMatrix;
}

void Object3D::translateLocal(const Vector3<double>& vector) {
    m_transformMatrix = m_transformMatrix * Matrix4<double>::translationMatrix(vector);
}

void Object3D::rotateWorld(const Vector3<double>& axis, float angle) {
    m_transformMatrix = Matrix4<double>::rotationMatrix(axis, angle) * m_transformMatrix;
}

void Object3D::rotateLocal(const Vector3<double>& axis, float angle) {
    m_transformMatrix = m_transformMatrix * Matrix4<double>::rotationMatrix(axis, angle);
}

void Object3D::rotateAroundWorldPoint(const Vector3<double>& axis, float angle, const Vector3<double>& point) {
    m_transformMatrix = Matrix4<double>::translationMatrix(point) * Matrix4<double>::rotationMatrix(axis, angle)
                       * Matrix4<double>::translationMatrix(-point) * m_transformMatrix;
}

void Object3D::rotateAroundLocalPoint(const Vector3<double>& axis, float angle, const Vector3<double>& worldPoint) {

    // Convert the world point into the local coordinate system
    Vector3 localPoint = m_transformMatrix.getInverse() * worldPoint;

    m_transformMatrix = m_transformMatrix * Matrix4<double>::translationMatrix(localPoint)
                       * Matrix4<double>::rotationMatrix(axis, angle)
                       * Matrix4<double>::translationMatrix(-localPoint);
}
