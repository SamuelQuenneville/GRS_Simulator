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

#include <string>
#include <vector>

#include "Mathematics/color.h"
#include "Mathematics/math.h"

struct Vertex {
    grs::Vec3f position;
    uint32_t color;
};

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

    // Numeric values shown in the GUI debug panel (copied so the GUI never
    // reads live simulation objects from the render thread)
    std::string     name;
    bool            isCenterOfMass = false;
    grs::Vec3d      position = grs::Vec3d::zeros();
    grs::Vec3d      velocity = grs::Vec3d::zeros();
    grs::Vec3d      wind     = grs::Vec3d::zeros();
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
    grs::Vec3d startPosition = grs::Vec3d::zeros();
    grs::Vec3d endPosition   = grs::Vec3d::zeros();
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
