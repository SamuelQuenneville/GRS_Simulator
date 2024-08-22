/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SYSTEMDEFINITIONS_H
#define GRS_SIMULATOR_SYSTEMDEFINITIONS_H

#pragma once

enum systemType {
    ONE_AIRCRAFT,
    ONE_AIRCRAFT_GROUNDED,
    TWO_AIRCRAFT_GROUNDED_V,
    TWO_AIRCRAFT,
    TWO_AIRCRAFT_FREE_V,
    TWO_AIRCRAFT_GROUNDED_Y,
    TWO_AIRCRAFT_FREE_Y,
};

struct systemConfiguration {
    systemType type;
    int hlcRate;
    int llcRate;
    bool windOn;
    bool tetherAerodynamicOn;
};

#endif //GRS_SIMULATOR_SYSTEMDEFINITIONS_H
