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

#include "tetherDefinitions.h"
#include "fixedWingDefinitions.h"
#include "payloadDefinitions.h"
#include "launcherDefinitions.h"

enum systemType {
    FREE,
    V,
    Y,
};

struct systemProperties {
    fixedWingProperties fixedWing{};
    tetherProperties tether{};
    payloadProperties payload{};
    launcherProperties launcher{};
};

#endif //GRS_SIMULATOR_SYSTEMDEFINITIONS_H
