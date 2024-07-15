/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TRANSFORM_H
#define GRS_SIMULATOR_TRANSFORM_H

#pragma once

#include "vector3.h"
#include "quaternion.h"

class Transform {

public:
    Transform();
    Transform(const Vector3<double>& position, const Matrix3<double>& orientation);
    Transform(const Vector3<double>& position, const Quaternion& orientation);

    [[nodiscard]] const Vector3<double>& getPosition() const {return m_position;};
    void setPosition(const Vector3<double>& newPosition) {m_position = newPosition;};

    [[nodiscard]] const Quaternion& getOrientation() const {return m_orientation;};
    void setOrientation(const Quaternion& newOrientation) {m_orientation = newOrientation;};

    void setToIdentity();

    // Set the transform from an OpenGL transform matrix
    void setFromOpenGL(double* openglMatrix);

    // Get the OpenGL matrix of the transform
    void getOpenGLMatrix(double* openglMatrix) const;

    // Return the inverse of the transform
    [[nodiscard]] Transform getInverse() const;

    // Return an interpolated transform
    static Transform interpolateTransforms(const Transform& oldTransform, const Transform& newTransform, double interpolationFactor);

    // Return the identity transform
    static Transform identity();

    [[nodiscard]] bool isValid() const;

    // operator
    Vector3<double> operator*(const Vector3<double>& vector) const;
    Transform operator*(const Transform& transform2) const;

    [[nodiscard]] std::string toString() const;

private:
    Vector3<double> m_position;
    Quaternion m_orientation;

};


#endif //GRS_SIMULATOR_TRANSFORM_H
