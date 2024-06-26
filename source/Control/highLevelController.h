/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_HIGHLEVELCONTROLLER_H
#define GRS_SIMULATOR_HIGHLEVELCONTROLLER_H

#include <cmath>
#include <memory>
#include <vector>

#include "Definitions/configurationDefinitions.h"

template <typename COMMANDS, typename STATES>
class HighLevelController {

public:
    virtual COMMANDS calculateInputs(const std::vector<STATES>& currentStates, const target& target) = 0;
    virtual ~HighLevelController() = default;

};


#endif //GRS_SIMULATOR_HIGHLEVELCONTROLLER_H
