/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WORLDPHYSICS_H
#define GRS_SIMULATOR_WORLDPHYSICS_H

#include "Mathematics/vector3.h"

struct worldPhysics {
    double gravity = 9.81;
    double airDensity = 1.225;
    double soundSpeed = 343.0;

    Vector3<double> wind = {0,0,0};

    void setGravity(const double newGravity) {gravity = newGravity;};
    void setAirDensity(const double newDensity) {airDensity = newDensity;};
    void updateWind(const Vector3<double> &newWind) {wind = newWind;};
};

#endif //GRS_SIMULATOR_WORLDPHYSICS_H
