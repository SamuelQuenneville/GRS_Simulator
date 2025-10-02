/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MATHUTILS_H
#define GRS_SIMULATOR_MATHUTILS_H

#pragma once
#include <cmath>
#include <cstdint>

namespace grs {

// Convert an angle in degrees to radians
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
[[nodiscard]] constexpr T degToRad(T degrees) noexcept {
    return degrees * static_cast<T>(M_PI / 180.0);
}

// Convert an angle in radians to degrees
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
[[nodiscard]] constexpr T radToDeg(T radians) noexcept {
    return radians * static_cast<T>(180.0 / M_PI);
}

template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
constexpr T wrapToPi(T angle) noexcept {
    angle = std::fmod(angle + static_cast<T>(M_PI), static_cast<T>(2.0 * M_PI));

    if (angle < T(0)) {
        angle += static_cast<T>(2.0 * M_PI);
    }

    return angle - static_cast<T>(M_PI);
}

template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
T unwrap(T prevUnwrapped, T newWrapped) {
    T delta = wrapToPi(newWrapped - prevUnwrapped);
    return prevUnwrapped + delta;
}


}

#endif //GRS_SIMULATOR_MATHUTILS_H
