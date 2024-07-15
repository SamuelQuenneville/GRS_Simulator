/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_QUATERNION_H
#define GRS_SIMULATOR_QUATERNION_H

#pragma once

#include <cassert>
#include "vector3.h"
#include "matrix3.h"

/*
 * This class represents a quaternion
 * We use the notation : q = (x*i, y*j, z*k, w)
 */
class Quaternion {

public:
    double x;
    double y;
    double z;
    double w;

    Quaternion();
    Quaternion(double newX, double newY, double newZ, double newW);
    Quaternion(const Vector3<double>& vector, double newW);

    // Create a unit quaternion from a rotation matrix
    explicit Quaternion(const Matrix3<double>& matrix);

    void setAllValues(double newX, double newY, double newZ, double newW);

    void setToZero();
    void setToIdentity();

    [[nodiscard]] Vector3<double> getVector() const;

    [[nodiscard]] double lengthSquare() const;
    [[nodiscard]] double length() const;

    void normalize();

    void inverse();

    [[nodiscard]] Quaternion getUnit() const;

    [[nodiscard]] Quaternion getConjugate() const;

    [[nodiscard]] Quaternion getInverse() const;

    // Return the orientation matrix corresponding to this quaternion
    [[nodiscard]] Matrix3<double> getMatrix() const;

    // Return the identity quaternion
    static Quaternion identity();

    // Return a quaternion constructed from Euler angles (in radians)
    static Quaternion fromEulerAngles(double angleX, double angleY, double angleZ);

    // Return a quaternion constructed from Euler angles (in radians)
    static Quaternion fromEulerAngles(const Vector3<double>& eulerAngles);

    // Dot product between two quaternions
    [[nodiscard]] double dot(const Quaternion& quaternion) const;

    // Compute the rotation angle (in radians) and the rotation axis
    void getRotationAngleAxis(double& angle, Vector3<double>& axis) const;

    // Return true if the values are not NAN OR INF
    [[nodiscard]] bool isFinite() const;

    // Return true if it is a unit quaternion
    [[nodiscard]] bool isUnit() const;

    // Return true if it is a valid quaternion
    [[nodiscard]] bool isValid() const;

    // Compute the spherical linear interpolation between two quaternions
    static Quaternion slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, double t);

    // Overloaded operator
    Quaternion operator+(const Quaternion& quaternion) const;
    Quaternion operator-(const Quaternion& quaternion) const;
    Quaternion& operator+=(const Quaternion& quaternion);
    Quaternion& operator-=(const Quaternion& quaternion);
    Quaternion operator*(double rhs) const;
    Quaternion operator*(const Quaternion& quaternion) const;
    Vector3<double> operator*(const Vector3<double>& point) const;
    bool operator==(const Quaternion& quaternion) const;

    // Return the string representation
    [[nodiscard]] std::string toString() const;

private:
    // Initialize the quaternion using Euler angles
    void m_initWithEulerAngles(double angleX, double angleY, double angleZ);
};


#endif //GRS_SIMULATOR_QUATERNION_H
