/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

/**
 * @file color.h
 * @brief Defines a simple RGBA color class with utility functions and presets.
 *
 * The Color class provides a compact representation of colors using four
 * floating-point channels (red, green, blue, alpha). It includes convenience
 * constructors, common predefined colors, and a basic color lookup method.
 */

#ifndef GRS_SIMULATOR_COLOR_H
#define GRS_SIMULATOR_COLOR_H

#pragma once

#include <stdexcept>

/**
 * @class Color
 * @brief Represents an RGBA color with utility methods and presets.
 *
 * Colors are stored as four floats (red, green, blue, alpha), each typically in
 * the range [0.0, 1.0]. The class provides constructors, predefined colors, and
 * a utility to retrieve colors by index.
 */
class Color {

public:
    float r = 1.f; ///< Red channel [0.0–1.0]
    float g = 1.f; ///< Green channel [0.0–1.0]
    float b = 1.f; ///< Blue channel [0.0–1.0]
    float a = 1.f; ///< Alpha (opacity) channel [0.0–1.0]

    /**
     * @brief Default constructor. Initializes to opaque white (1,1,1,1).
     */
    constexpr Color() = default;

    /**
     * @brief Construct a color from individual channel values.
     * @param r_ Red channel in [0.0–1.0]
     * @param g_ Green channel in [0.0–1.0]
     * @param b_ Blue channel in [0.0–1.0]
     * @param a_ Alpha channel in [0.0–1.0], defaults to 1.0 (opaque)
     */
    constexpr Color(const float r_, const float g_, const float b_, const float a_ = 1.f) noexcept
        : r(r_), g(g_), b(b_), a(a_) {}

    /** @brief Copy constructor (defaulted) */
    constexpr Color(const Color&) noexcept = default;

    /** @brief Copy assignment operator (defaulted) */
    constexpr Color& operator=(const Color&) noexcept = default;

    /**
     * @enum HexColor
     * @brief Predefined colors in hexadecimal RGB format.
     */
    enum class HexColor {
        RED     = 0xff0000,
        WARM_RED = 0xC85A5A,
        GREEN   = 0x00ff00,
        WARM_GREEN = 0x5FAF5F,
        BLUE    = 0x0000ff,
        GRAY    = 0xA8A8A8,
        BLACK   = 0x000000,
        WHITE   = 0xffffff,
        YELLOW  = 0xffff00,
        MAGENTA = 0xff00ff,
        CYAN    = 0x00ffff,
        GRASS   = 0x325032,
    };

    /** @name Predefined Colors
     * Convenience constructors for common colors.
     * @{
     */
    static constexpr Color white()  { return {1.0f, 1.0f, 1.0f, 1.0f}; } ///< White (opaque)
    static constexpr Color red()    { return {1.0f, 0.0f, 0.0f, 1.0f}; } ///< Red (opaque)
    static constexpr Color blue()   { return {0.0f, 0.0f, 1.0f, 1.0f}; } ///< Blue (opaque)
    static constexpr Color yellow() { return {1.0f, 1.0f, 0.0f, 1.0f}; } ///< Yellow (opaque)

    static constexpr Color backgroundColor() { return {0.7f, 0.7f, 0.7f, 1.0f}; } ///< Default background gray
    static constexpr Color floorColor()      { return {0.78f, 0.78f, 0.78f, 1.0f}; }      ///< Default floor gray
    /** @} */

    /**
     * @brief Retrieve a color by index.
     *
     * Supported indices:
     * - 1 → Red
     * - 2 → Yellow
     * - 3 → Blue
     * - 4 → White
     *
     * @param index Integer index (1–4).
     * @return Corresponding Color.
     * @throws std::out_of_range if the index is outside [1,4].
     */
    [[nodiscard]] static Color color(const int index) {
        switch (index) {
            case 1: return red();
            case 2: return yellow();
            case 3: return blue();
            case 4: return white();
            default: throw std::out_of_range("Color index is out of range");
        }
    }
};

#endif //GRS_SIMULATOR_COLOR_H
