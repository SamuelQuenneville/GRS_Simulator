/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHERDEFINITIONS_H
#define GRS_SIMULATOR_TETHERDEFINITIONS_H

#pragma once

#include "Mathematics/vector3.h"

/*
 *   TETHERS STRUCTURES
 */

struct tetherProperties {
    float linearMass;  // kg/m
    float length;      // m
    float diameter;    // m
    float rigidity;    // spring model k value
    float damping;     // spring model b value
    int   numSegments; // number of segments use to compute aerodynamic forces
};

struct tetherLogInfos {
    double deformation{};           // m
    Vector3<double> forceSpring;    // N
    Vector3<double> forceAero;      // N
};

#endif //GRS_SIMULATOR_TETHERDEFINITIONS_H
