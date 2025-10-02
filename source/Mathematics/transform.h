/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_b_H
#define GRS_SIMULATOR_b_H

#pragma once

#include "quaternion.h"

namespace grs {

template <typename T>
struct Transform {
    Vector<T,3> position{T(0), T(0), T(0)};
    Quaternion<T> orientation{Quaternion<T>::identity()};

    constexpr Transform() noexcept = default;
    constexpr Transform(const Vector<T,3>& pos, const Quaternion<T>& ori) noexcept
        : position(pos), orientation(ori) {}

    [[nodiscard]] static Transform identity() noexcept {
        return Transform(Vector<T,3>::zeros(), Quaternion<T>::identity());
    }

    // Convert to a homogeneous 4x4 matrix (row-major)
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

    [[nodiscard]] static Transform interpolate(const Transform& a, const Transform& b, T t) {
        return Transform(grs::lerp(a.position, b.position, t),
                            grs::slerp(a.orientation, b.orientation, t));
    }
};

template <typename T>
[[nodiscard]] constexpr Transform<T> operator*(const Transform<T>& a, const Transform<T>& b) noexcept {
    const auto position = a.position + a.orientation * b.position;
    const auto orientation = a.orientation * b.orientation;

    return {position, orientation};
}

}

#endif //GRS_SIMULATOR_b_H
