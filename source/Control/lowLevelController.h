/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOWLEVELCONTROLLER_H
#define GRS_SIMULATOR_LOWLEVELCONTROLLER_H

#include <cmath>

template <typename INPUTS, typename STATES, typename COMMANDS>
class LowLevelController {

public:
    virtual INPUTS calculateAircraftControlInputs(const STATES& currentStates, const COMMANDS& commands, double desiredThrust) = 0;
    virtual ~LowLevelController() = default;

};


#endif //GRS_SIMULATOR_LOWLEVELCONTROLLER_H
