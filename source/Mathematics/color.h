/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_COLOR_H
#define GRS_SIMULATOR_COLOR_H

#pragma once

class Color {

public:
    float r = 1.f;
    float g = 1.f;
    float b = 1.f;
    float a = 1.f;

    constexpr Color() = default;
    constexpr Color(const float r_, const float g_, const float b_, const float a_ = 1.f) noexcept
        : r(r_), g(g_), b(b_), a(a_) {}

    constexpr Color(const Color&) noexcept = default;
    constexpr Color& operator=(const Color&) noexcept = default;

    enum class HexColor {
        RED     = 0xff0000,
        GREEN   = 0x00ff00,
        BLUE    = 0x0000ff,
        BLACK   = 0x000000,
        WHITE   = 0xffffff,
        YELLOW  = 0xffff00,
        MAGENTA = 0xff00ff,
        CYAN    = 0x00ffff,
    };

    // Return the white color
    static constexpr Color white()  {return {1.0f, 1.0f, 1.0f, 1.0f};}
    static constexpr Color red()    {return {1.0f, 0.0f, 0.0f, 1.0f};}
    static constexpr Color blue()   {return {0.0f, 0.0f, 1.0f, 1.0f};}
    static constexpr Color yellow() {return {1.0f, 1.0f, 0.0f, 1.0f};}

    static constexpr Color backgroundColor() {return {0.298f, 0.298f, 0.32157f, 1.0f};}
    static constexpr Color floorColor() {return {0.78f, 0.78f, 0.78f, 1.0f};}

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
