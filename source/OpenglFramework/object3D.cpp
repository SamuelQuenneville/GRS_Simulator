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

const grs::Matrix4d& Object3D::getTransformMatrix() const {
    return m_transformMatrix;
}

void Object3D::setTransformMatrix(const grs::Matrix4d& matrix) {
    m_transformMatrix = matrix;
}

void Object3D::setToIdentity() {
    m_transformMatrix = grs::Matrix4d::identity();
}

grs::Vec3d Object3D::getOrigin() const {
    return m_transformMatrix * grs::Vec3d::zeros();
}

void Object3D::translateWorld(const grs::Vec3d& vector) {
    m_transformMatrix = grs::makeTranslation(vector) * m_transformMatrix;
}

void Object3D::translateLocal(const grs::Vec3d& vector) {
    m_transformMatrix = m_transformMatrix * grs::makeTranslation(vector);
}

void Object3D::rotateWorld(const grs::Vec3d& axis, const double angle) {
    m_transformMatrix = grs::makeRotation(axis, angle) * m_transformMatrix;
}

void Object3D::rotateLocal(const grs::Vec3d& axis, const double angle) {
    m_transformMatrix = m_transformMatrix * grs::makeRotation(axis, angle);
}

void Object3D::rotateAroundWorldPoint(const grs::Vec3d& axis, const double angle, const grs::Vec3d& point) {
    m_transformMatrix = grs::makeTranslation(point) * grs::makeRotation(axis, angle)
                       * grs::makeTranslation(-point) * m_transformMatrix;
}

void Object3D::rotateAroundLocalPoint(const grs::Vec3d& axis, const double angle, const grs::Vec3d& worldPoint) {

    // Convert the world point into the local coordinate system
    const grs::Vec3d localPoint = m_transformMatrix.inverse() * worldPoint;

    m_transformMatrix = m_transformMatrix * grs::makeTranslation(localPoint)
                       * grs::makeRotation(axis, angle)
                       * grs::makeTranslation(-localPoint);
}
