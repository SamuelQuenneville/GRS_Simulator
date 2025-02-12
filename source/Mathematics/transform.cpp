/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "transform.h"

Transform::Transform()
    : m_position(Vector3<double>::zeros())
    , m_orientation(Quaternion::identity())
{

}

Transform::Transform(const Vector3<double>& position, const Matrix3<double>& orientation)
    : m_position(position)
    , m_orientation(Quaternion(orientation))
{

}

Transform::Transform(const Vector3<double>& position, const Quaternion& orientation)
    : m_position(position)
    , m_orientation(orientation)
{

}

void Transform::setToIdentity() {
    m_position = Vector3<double>::zeros();
    m_orientation = Quaternion::identity();
}

void Transform::getOpenGLMatrix(double* openglMatrix) const {
    const Matrix3<double>& matrix = m_orientation.getMatrix();
    openglMatrix[0] = matrix[0][0];
    openglMatrix[1] = matrix[1][0];
    openglMatrix[2] = matrix[2][0];
    openglMatrix[3] = 0.0;
    openglMatrix[4] = matrix[0][1];
    openglMatrix[5] = matrix[1][1];
    openglMatrix[6] = matrix[2][1];
    openglMatrix[7] = 0.0;
    openglMatrix[8] = matrix[0][2];
    openglMatrix[9] = matrix[1][2];
    openglMatrix[10] = matrix[2][2];
    openglMatrix[11] = 0.0;
    openglMatrix[12] = m_position.x();
    openglMatrix[13] = m_position.y();
    openglMatrix[14] = m_position.z();
    openglMatrix[15] = 1.0;
}

Transform Transform::getInverse() const {
    const Quaternion& invQuaternion = m_orientation.getInverse();
    return {invQuaternion * (-m_position), invQuaternion};
}

Transform Transform::interpolateTransforms(const Transform& oldTransform, const Transform& newTransform, const double interpolationFactor) {
    Vector3 interPosition = oldTransform.m_position * (1.0 - interpolationFactor) + newTransform.m_position * interpolationFactor;
    Quaternion interOrientation = Quaternion::slerp(oldTransform.m_orientation, newTransform.m_orientation,interpolationFactor);

    return {interPosition, interOrientation};
}

Transform Transform::identity() {
    return {Vector3<double>::zeros(), Quaternion::identity()};
}

Vector3<double> Transform::operator*(const Vector3<double>& vector) const {
    return (m_orientation * vector) + m_position;
}

// Operator of multiplication of a transform with another one
Transform Transform::operator*(const Transform& transform2) const {

    // The following code is equivalent to this
    // return Transform(m_position + m_orientation * transform2.m_position,
    //                 m_orientation * transform2.m_orientation);

    const double prodX = m_orientation.w * transform2.m_position.x() + m_orientation.y * transform2.m_position.z() - m_orientation.z * transform2.m_position.y();
    const double prodY = m_orientation.w * transform2.m_position.y() + m_orientation.z * transform2.m_position.x() - m_orientation.x * transform2.m_position.z();
    const double prodZ = m_orientation.w * transform2.m_position.z() + m_orientation.x * transform2.m_position.y() - m_orientation.y * transform2.m_position.x();
    const double prodW = -m_orientation.x * transform2.m_position.x() - m_orientation.y * transform2.m_position.y() - m_orientation.z * transform2.m_position.z();

    Vector3<double> position = {m_position.x() + m_orientation.w * prodX - prodY * m_orientation.z + prodZ * m_orientation.y - prodW * m_orientation.x,
                                m_position.y() + m_orientation.w * prodY - prodZ * m_orientation.x + prodX * m_orientation.z - prodW * m_orientation.y,
                                m_position.z() + m_orientation.w * prodZ - prodX * m_orientation.y + prodY * m_orientation.x - prodW * m_orientation.z};

    Quaternion orientation = {m_orientation.w * transform2.m_orientation.x + transform2.m_orientation.w * m_orientation.x
                              + m_orientation.y * transform2.m_orientation.z - m_orientation.z * transform2.m_orientation.y,
                              m_orientation.w * transform2.m_orientation.y + transform2.m_orientation.w * m_orientation.y
                              + m_orientation.z * transform2.m_orientation.x - m_orientation.x * transform2.m_orientation.z,
                              m_orientation.w * transform2.m_orientation.z + transform2.m_orientation.w * m_orientation.z
                              + m_orientation.x * transform2.m_orientation.y - m_orientation.y * transform2.m_orientation.x,
                              m_orientation.w * transform2.m_orientation.w - m_orientation.x * transform2.m_orientation.x
                              - m_orientation.y * transform2.m_orientation.y - m_orientation.z * transform2.m_orientation.z};

    return {position, orientation};
}

Transform& Transform::operator=(const Transform& transform2) {
    m_position = transform2.getPosition();
    m_orientation = transform2.getOrientation();
    return *this;
}