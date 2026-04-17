/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_RENDEROBJECTDEFINITIONS_H
#define GRS_SIMULATOR_RENDEROBJECTDEFINITIONS_H

#pragma once

#include "Mathematics/color.h"
#include "Mathematics/math.h"

enum RenderObjectType {
    UNDEFINED,
    PAYLOAD,
    AIRCRAFT
};

/*
 *   FRAME AXIS STRUCTURES
 */

struct frameRenderData {
    grs::TransformD transform     = grs::TransformD::identity();
    grs::Vec3d      forceAtOrigin = grs::Vec3d::zeros();
    Color::HexColor forceColor    = Color::HexColor::BLACK;
};

/*
 *   BODY STRUCTURES
 */

struct bodyRenderData {
    RenderObjectType type = RenderObjectType::UNDEFINED;
    int id;
    grs::TransformD transform = grs::TransformD::identity();
    std::vector<frameRenderData> frames;
};

/*
 *   TETHER STRUCTURES
 */

struct tetherRenderData {
    grs::Vec3f startPosition = grs::Vec3f::zeros();
    grs::Vec3f endPosition   = grs::Vec3f::zeros();
};

/*
 *   WORLD STRUCTURES
 */

struct systemRenderData {
    double elapsedTime = 0.0;
    std::vector<bodyRenderData> bodies;
    std::vector<tetherRenderData> tethers;
};

#endif //GRS_SIMULATOR_RENDEROBJECTDEFINITIONS_H
