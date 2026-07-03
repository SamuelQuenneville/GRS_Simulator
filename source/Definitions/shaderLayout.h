/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SHADERLAYOUT_H
#define GRS_SIMULATOR_SHADERLAYOUT_H

#pragma once

#include <nanogui/opengl.h>

namespace VertexLayout
{
    inline constexpr GLuint Position = 0;
    inline constexpr GLuint Normal   = 1;
    inline constexpr GLuint TexCoord = 2;
    inline constexpr GLuint Color    = 3;
}

#endif //GRS_SIMULATOR_SHADERLAYOUT_H
