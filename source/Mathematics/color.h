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
    // RGBA color components
    float r;
    float g;
    float b;
    float a;


    Color() : r(1), g(1), b(1), a(1) {}
    Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
    Color(const Color& color) : r(color.r), g(color.g), b(color.b), a(color.a) {}

    ~Color() = default;

    // Return the white color
    static Color white()  {return {1.0f, 1.0f, 1.0f, 1.0f};}
    static Color red()    {return {1.0f, 0.0f, 0.0f, 1.0f};}
    static Color blue()   {return {0.0f, 0.0f, 1.0f, 1.0f};}
    static Color yellow() {return {1.0f, 1.0f, 0.0f, 1.0f};}

    static Color backgroundColor() {return {0.298f, 0.298f, 0.32157f, 1.0f};}
    static Color floorColor() {return {0.78f, 0.78f, 0.78f, 1.0f};}

    // = operator
    Color& operator=(const Color& color) {
        if (&color != this) {
            r = color.r;
            g = color.g;
            b = color.b;
            a = color.a;
        }
        return *this;
    }
};

#endif //GRS_SIMULATOR_COLOR_H
