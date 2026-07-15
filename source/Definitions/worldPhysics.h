/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WORLDPHYSICS_H
#define GRS_SIMULATOR_WORLDPHYSICS_H

#pragma once

struct worldPhysics {
    float gravity = 9.81;
    float airDensity = 1.225;
    float soundSpeed = 343.0;

    [[maybe_unused]] void setGravity(const float newGravity) {gravity = newGravity;};
    [[maybe_unused]] void setAirDensity(const float newDensity) {airDensity = newDensity;};
};

#endif //GRS_SIMULATOR_WORLDPHYSICS_H
