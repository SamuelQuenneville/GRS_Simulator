/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_QUATERNION2_H
#define GRS_SIMULATOR_QUATERNION2_H

#pragma once
#include <cassert>

#include "matrix.h"

namespace grs {

template<typename T>
struct Quaternion {
    T x{}, y{}, z{}, w{}; // default = identity

    // Constructor
    constexpr Quaternion() = default;
    constexpr Quaternion(const T x_, const T y_, const T z_, const T w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    // += operator
    constexpr Quaternion& operator+=(const Quaternion& q) noexcept {
        x += q.x;
        y += q.y;
        z += q.z;
        w += q.w;
        return *this;
    }

    // -= operator
    constexpr Quaternion& operator-=(const Quaternion& q) noexcept {
        x -= q.x;
        y -= q.y;
        z -= q.z;
        w -= q.w;
        return *this;
    }

    // Identity
    [[nodiscard]] static constexpr Quaternion identity() noexcept {
        return {T(0), T(0), T(0), T(1)};
    }

    // Length
    [[nodiscard]] constexpr T lengthSquared() const noexcept {
        return x*x + y*y + z*z + w*w;
    }

    [[nodiscard]] constexpr T length() const noexcept {
        return std::sqrt(lengthSquared());
    }

    // Conjugate & Inverse
    [[nodiscard]] constexpr Quaternion conjugate() const noexcept {
        return {-x, -y, -z, w};
    }

    [[nodiscard]] Quaternion inverse() const noexcept {
        const T ls = lengthSquared();
        assert(ls > T(0));
        return conjugate() * (T(1) / ls);
    }

    // NED to Y up
    [[nodiscard]] Quaternion nedToYup() const noexcept {
        const T c = std::cos(M_PI_4);
        const Quaternion correction = {c, T(0), T(0), c};

        return correction * (*this);
    }

    // Y up to NED
    [[nodiscard]] Quaternion yUpToNed() const noexcept {
        const T c = std::cos(M_PI_4);
        const Quaternion correction = {-c, T(0), T(0), c};

        return correction * (*this);
    }

};

// Operators
template <typename T>
constexpr Quaternion<T> operator+(const Quaternion<T>& q1, const Quaternion<T>& q2) noexcept {
    return {q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w};
}

template <typename T>
constexpr Quaternion<T> operator-(const Quaternion<T>& q1, const Quaternion<T>& q2) noexcept {
    return {q1.x - q2.x, q1.y - q2.y, q1.z - q2.z, q1.w - q2.w};
}

template <typename T>
constexpr Quaternion<T> operator*(const Quaternion<T>& q, const T scalar) noexcept {
    return {q.x * scalar, q.y * scalar, q.z * scalar, q.w * scalar};
}

template <typename T>
constexpr Quaternion<T> operator*(const Quaternion<T>& q1, const Quaternion<T>& q2) noexcept {
    /* The following code is equivalent to this
     * return Quaternion(w * quaternion.w - getVectorV().dot(quaternion.getVectorV()),
     * w * quaternion.getVectorV() + quaternion.w * getVectorV() +
     * getVectorV().cross(quaternion.getVectorV()));
     */

    return {q1.w * q2.x + q2.w * q1.x + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y + q2.w * q1.y + q1.z * q2.x - q1.x * q2.z,
            q1.w * q2.z + q2.w * q1.z + q1.x * q2.y - q1.y * q2.x,
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z};
}

template <typename T>
constexpr Vector<T,3> operator*(const Quaternion<T>& q, const Vector<T,3>& point) noexcept {
    /* The following code is equivalent to this
     * Quaternion p(point.x, point.y, point.z, 0.0);
     * return (((*this) * p) * getConjugate()).getVectorV();
    */

    const T prodX = q.w * point[0] + q.y * point[2] - q.z * point[1];
    const T prodY = q.w * point[1] + q.z * point[0] - q.x * point[2];
    const T prodZ = q.w * point[2] + q.x * point[1] - q.y * point[0];
    const T prodW = -q.x * point[0] - q.y * point[1] - q.z * point[2];

    return {q.w * prodX - prodY * q.z + prodZ * q.y - prodW * q.x,
            q.w * prodY - prodZ * q.x + prodX * q.z - prodW * q.y,
            q.w * prodZ - prodX * q.y + prodY * q.x - prodW * q.z};
}

// Normalize
template <typename T>
[[nodiscard]] constexpr Quaternion<T> normalize(const Quaternion<T>& q) {
    const T len = q.length();
    return (len > T(0)) ? (q * (T(1)/len)) : q;
}

// Dot
template <typename T>
[[nodiscard]] constexpr T dot(const Quaternion<T>& q1, const Quaternion<T>& q2) noexcept {
    return q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
}

// Euler angles to quaternion
template <typename T>
[[nodiscard]] constexpr Quaternion<T> eulerToQuat(const T roll, const T pitch, const T yaw) {
    T angle = roll * 0.5;
    T sr = std::sin(angle);
    T cr = std::cos(angle);

    angle = pitch * 0.5;
    T sp = std::sin(angle);
    T cp = std::cos(angle);

    angle = yaw * 0.5;
    T sy = std::sin(angle);
    T cy = std::cos(angle);

    Quaternion<T> q{};
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.w = cr * cp * cy + sr * sp * sy;

    return normalize(q);
}

template <typename T>
[[nodiscard]] constexpr Quaternion<T> eulerToQuat(const Vector<T,3>& euler) {
    T angle = euler[0] * 0.5;
    T sr = std::sin(angle);
    T cr = std::cos(angle);

    angle = euler[1] * 0.5;
    T sp = std::sin(angle);
    T cp = std::cos(angle);

    angle = euler[2] * 0.5;
    T sy = std::sin(angle);
    T cy = std::cos(angle);

    Quaternion<T> q{};
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.w = cr * cp * cy + sr * sp * sy;

    return normalize(q);
}

// Quaternion to euler angles
template <typename T>
[[nodiscard]] constexpr Vector<T,3> quatToEuler(const Quaternion<T>& q) noexcept {

    // Rotation matrix
    //     | R11  R12  R13 |   | 1 - 2*(y*y + z*z)    2*(x*y - w*z)      2*(x*z + w*y)   |
    // R = | R21  R22  R23 | = |   2*(x*y + w*z)    1 - 2*(x*x + z*z)    2*(y*z - w*x)   |
    //     | R31  R32  R33 |   |   2*(x*z - w*y)      2*(y*z + w*x)    1 - 2*(x*x + y*y) |

    const T R32 = 2.0 * (q.y * q.z + q.w * q.x);
    const T R33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    T roll = std::atan2(R32, R33);

    const T negativeR31 = 2.0 * (q.w * q.y - q.z * q.x);
    T pitch;
    if (std::fabs(negativeR31) >= 1.0) {
        pitch = std::copysign(M_PI_2, negativeR31);
    } else {
        pitch = std::asin(negativeR31);
    }

    const T R21 = 2.0 * (q.x * q.y + q.w * q.z);
    const T R11 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    T yaw = std::atan2(R21, R11);

    return {roll + M_PI_2, pitch, yaw};
}

// Quaternion to rotation matrix
template <typename T>
[[nodiscard]] constexpr Matrix<T,3> quatToRotMatrix(const Quaternion<T>& q) noexcept {
    const T lenSq = q.lengthSquared();
    T s = T(0);

    if (lenSq > T(0)) {
        s = T(2) / lenSq;
    }

    // Computations used for optimization (fewer multiplications)
    const T xs = q.x*s;
    const T ys = q.y*s;
    const T zs = q.z*s;
    const T wxs = q.w*xs;
    const T wys = q.w*ys;
    const T wzs = q.w*zs;
    const T xxs = q.x*xs;
    const T xys = q.x*ys;
    const T xzs = q.x*zs;
    const T yys = q.y*ys;
    const T yzs = q.y*zs;
    const T zzs = q.z*zs;

    Matrix<T,3> m{};
    m(0,0) = 1.0 - yys - zzs; m(0,1) =  xys - wzs;       m(0,2) = xzs + wys;
    m(1,0) = xys + wzs;       m(1,1) =  1.0 - xxs - zzs; m(1,2) = yzs-wxs;
    m(2,0) = xzs-wys;         m(2,1) = yzs + wxs;        m(2,2) = 1.0 - xxs - yys;
    return m;
}

// Rotation matrix to quaternion
template <typename T>
[[nodiscard]] constexpr Quaternion<T> rotMatrixToQuat(const Matrix<T,3>& m) noexcept {
    const T tr = m.trace();

    Quaternion<T> q{};
    T r,s;

    if (tr < T(0)) {
        if (m(1,1) > m(0,0)) {
            if (m(2,2) > m(1,1)) {
                r = std::sqrt(m(2,2) - m(0,0) - m(1,1) + T(1));
                s = 0.5 / r;

                // Compute the quaternion
                q.x = (m(2,0) + m(0,2)) * s;
                q.y = (m(1,2) + m(2,1)) * s;
                q.z = 0.5 * r;
                q.w = (m(1,0) + m(0,1)) * s;
            } else {
                r = std::sqrt(m(1,1) - m(2,2) - m(0,0) + T(1));
                s = 0.5 / r;

                // Compute the quaternion
                q.x = (m(0,1) + m(1,0)) * s;
                q.y = 0.5 * r;
                q.z = (m(1,2) + m(2,1)) * s;
                q.w = (m(0,2) - m(2,0)) * s;
            }
        } else if (m(2,2) > m(0,0)) {
            r = std::sqrt(m(2,2) - m(0,0) - m(1,1) + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            q.x = (m(2,0) + m(0,2)) * s;
            q.y = (m(1,2) + m(2,1)) * s;
            q.z = 0.5 * r;
            q.w = (m(1,0) - m(0,1)) * s;
        } else {
            r = std::sqrt(m(0,0) - m(1,1) - m(2,2) + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            q.x = 0.5 * r;
            q.y = (m(0,1) + m(1,0)) * s;
            q.z = (m(2,0) + m(0,2)) * s;
            q.w = (m(2,1) - m(1,2)) * s;
        }
    } else {
        r = std::sqrt(tr + T(1));
        s = 0.5 / r;

        // Compute the quaternion
        q.x = (m(2,1) - m(1,2)) * s;
        q.y = (m(0,2) - m(2,0)) * s;
        q.z = (m(1,0) - m(0,1)) * s;
        q.w = 0.5 * r;
    }

    return q;
}

// Spherical Linear Interpolation, t must be between 0 and 1
template <typename T>
[[nodiscard]] constexpr Quaternion<T> slerp(const Quaternion<T>& q1, const Quaternion<T>& q2, const T t) {
    int invert = 1;

    // Compute cos(theta) using the quaternion scalar product
    T cosineTheta = dot(q1, q2);

    // Take care of the sign of cosineTheta
    if (cosineTheta < T(0)) {
        cosineTheta = -cosineTheta;
        invert = -1;
    }

    // Because of precision, if cos(theta) is nearly 1,
    // therefore theta is nearly 0, and we can write
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    if(T(1) - cosineTheta < std::numeric_limits<T>::epsilon()) {
        return q1 * (T(1) - t) + q2 * (t * invert);
    }

    // Compute the theta angle
    const T theta = std::acos(cosineTheta);

    // Compute sin(theta)
    const T sineTheta = std::sin(theta);

    // Compute the two coefficients that are in the spherical linear interpolation formula
    const T coeff1 = std::sin((T(1) - t)*theta) / sineTheta;
    const T coeff2 = std::sin(t*theta) / sineTheta * invert;

    // Compute and return the interpolated quaternion
    return q1 * coeff1 + q2 * coeff2;
}

}

#endif //GRS_SIMULATOR_QUATERNION2_H
