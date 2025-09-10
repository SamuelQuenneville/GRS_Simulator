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
    // Create a unit quaternion from a rotation matrix
    explicit Quaternion(const Matrix3<double>& matrix);

    void setAllValues(double newX, double newY, double newZ, double newW);

    [[nodiscard]] double lengthSquare() const;
    [[nodiscard]] double length() const;

    void normalize();

    [[nodiscard]] Quaternion getInverse() const;

    // Return the orientation matrix corresponding to this quaternion
    [[nodiscard]] Matrix3<double> getMatrix() const;

    // Return the identity quaternion
    static Quaternion identity();

    // Return a quaternion constructed from Euler angles (in radians)
    static Quaternion fromEulerAngles(double roll, double pitch, double yaw);

    Vector3<double> eulerAngles() const;

    // Dot product between two quaternions
    [[nodiscard]] double dot(const Quaternion& quaternion) const;

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

private:
    // Initialize the quaternion using Euler angles
    void m_initWithEulerAngles(double roll, double pitch, double yaw);
};

#endif //GRS_SIMULATOR_QUATERNION_H
