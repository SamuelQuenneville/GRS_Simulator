/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file transform.h
 * @brief Defines the Transform template struct and related operations.
 *
 * This header provides the definition of the Transform struct, which represents
 * a 3D rigid body transform consisting of a position (translation) and an
 * orientation (rotation stored as a quaternion). It supports conversion to
 * homogeneous transformation matrices, interpolation between transforms, and
 * composition of transforms using the multiplication operator.
 *
 * The Transform struct is useful for describing object placement, camera poses,
 * and coordinate frame transformations in the GRS simulation environment.
 */

#ifndef GRS_SIMULATOR_b_H
#define GRS_SIMULATOR_b_H

#pragma once

#include "quaternion.h"

namespace grs {

/**
 * @brief Represents a 3D rigid body transform (position + orientation).
 *
 * @tparam T Numeric type (e.g., float, double).
 *
 * A Transform consists of:
 * - A 3D position vector.
 * - An orientation represented as a quaternion.
 *
 * It supports conversion to a homogeneous 4×4 transformation matrix,
 * interpolation (LERP for translation, SLERP for rotation),
 * and transform composition via operator*.
 */
template <typename T>
struct Transform {
    /** @brief Translation component. */
    Vector<T,3> position{T(0), T(0), T(0)};

    /** @brief Rotation component. */
    Quaternion<T> orientation{Quaternion<T>::identity()};

    /** @brief Default constructor (identity transform). */
    constexpr Transform() noexcept = default;

    /**
     * @brief Construct a Transform from position and orientation.
     * @param pos Translation vector.
     * @param ori Orientation quaternion.
     */
    constexpr Transform(const Vector<T,3>& pos, const Quaternion<T>& ori) noexcept
        : position(pos), orientation(ori) {}

    /**
     * @brief Return an identity transform (zero translation, unit quaternion).
     * @return Identity Transform.
     */
    [[nodiscard]] static Transform identity() noexcept {
        return Transform(Vector<T,3>::zeros(), Quaternion<T>::identity());
    }

    /**
     * @brief Convert this transform to a homogeneous 4×4 matrix (row-major).
     *
     * The resulting matrix combines rotation (upper-left 3×3)
     * and translation (last column).
     *
     * @return 4×4 transformation matrix.
     */
    [[nodiscard]] Matrix<T,4> toMatrix4() const noexcept {
        Matrix<T,4> M{};
        auto R = quatToRotMatrix(orientation);

        // Upper-left 3x3 is rotation
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                M(r, c) = R(r, c);
            }
        }

        // Last column = position
        M(0, 3) = position[0];
        M(1, 3) = position[1];
        M(2, 3) = position[2];

        // Bottom row
        M(3, 0) = T(0);
        M(3, 1) = T(0);
        M(3, 2) = T(0);
        M(3, 3) = T(1);

        return M;
    }

    /**
     * @brief Interpolate between two transforms.
     *
     * Performs linear interpolation (LERP) on position and spherical linear
     * interpolation (SLERP) on orientation.
     *
     * @param a First transform.
     * @param b Second transform.
     * @param t Interpolation factor (0 → a, 1 → b).
     * @return Interpolated transform.
     */
    [[nodiscard]] static Transform interpolate(const Transform& a, const Transform& b, T t) {
        return Transform(grs::lerp(a.position, b.position, t),
                            grs::slerp(a.orientation, b.orientation, t));
    }
};

/**
 * @brief Compose two transforms via multiplication.
 *
 * The resulting transform applies `b` in the local space of `a`.
 *
 * @param a First transform (applied first).
 * @param b Second transform (applied after a).
 * @return Combined transform.
 */
template <typename T>
[[nodiscard]] constexpr Transform<T> operator*(const Transform<T>& a, const Transform<T>& b) noexcept {
    const auto position = a.position + a.orientation * b.position;
    const auto orientation = a.orientation * b.orientation;

    return {position, orientation};
}

}

#endif //GRS_SIMULATOR_b_H
