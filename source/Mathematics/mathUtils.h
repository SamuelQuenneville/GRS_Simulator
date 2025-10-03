/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file mathUtils.h
 * @brief Utility math functions for angle conversions and wrapping.
 *
 * This header provides common mathematical helper functions.
 */

#ifndef GRS_SIMULATOR_MATHUTILS_H
#define GRS_SIMULATOR_MATHUTILS_H

#pragma once
#include <cmath>
#include <cstdint>

namespace grs {

/**
 * @brief Convert an angle in degrees to radians.
 *
 * @tparam T Floating-point type (float, double, long double).
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
[[nodiscard]] constexpr T degToRad(T degrees) noexcept {
    return degrees * static_cast<T>(M_PI / 180.0);
}

/**
 * @brief Convert an angle in radians to degrees.
 *
 * @tparam T Floating-point type (float, double, long double).
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
[[nodiscard]] constexpr T radToDeg(T radians) noexcept {
    return radians * static_cast<T>(180.0 / M_PI);
}

/**
 * @brief Wrap an angle to the range [-π, π].
 *
 * This function ensures that the given angle lies within the canonical range
 * of [-π, π], which is commonly used in robotics and navigation.
 *
 * @tparam T Floating-point type (float, double, long double).
 * @param angle Input angle in radians.
 * @return Wrapped angle in radians.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
constexpr T wrapToPi(T angle) noexcept {
    angle = std::fmod(angle + static_cast<T>(M_PI), static_cast<T>(2.0 * M_PI));

    if (angle < T(0)) {
        angle += static_cast<T>(2.0 * M_PI);
    }

    return angle - static_cast<T>(M_PI);
}

/**
 * @brief Unwrap a wrapped angle to maintain continuity.
 *
 * Given a previously unwrapped angle and a new wrapped angle,
 * this function adjusts the new angle so that it remains continuous
 * with the previous value, avoiding sudden jumps at ±π.
 *
 * @tparam T Floating-point type (float, double, long double).
 * @param prevUnwrapped The previous unwrapped angle (radians).
 * @param newWrapped The new wrapped angle (radians).
 * @return Unwrapped angle (radians), continuous with prevUnwrapped.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
constexpr T unwrap(T prevUnwrapped, T newWrapped) {
    T delta = wrapToPi(newWrapped - prevUnwrapped);
    return prevUnwrapped + delta;
}


}

#endif //GRS_SIMULATOR_MATHUTILS_H
