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
#include <map>

#include "Definitions/configurationDefinitions.h"
#include "Definitions/controllerGains.h"
#include "Definitions/payloadDefinitions.h"
#include "Log/logger.h"

template <typename COMMANDS, typename STATES>
class HighLevelController {

public:
    virtual std::map<int, COMMANDS> calculateInputs(const std::vector<int>& ids, const std::vector<STATES>& currentStates, const target& target) = 0;
    virtual std::map<int, COMMANDS> calculateInputs(const std::vector<int>& ids, const std::vector<STATES>& currentStates, const payloadStates& payloadStates, const target& target) = 0;
    virtual ~HighLevelController() = default;
};


#endif //GRS_SIMULATOR_HIGHLEVELCONTROLLER_H
