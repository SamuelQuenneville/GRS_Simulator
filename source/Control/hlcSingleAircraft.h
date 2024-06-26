/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_HLCSINGLEAIRCRAFT_H
#define GRS_SIMULATOR_HLCSINGLEAIRCRAFT_H

#include "Control/highLevelController.h"
#include "Control/groundspeedController.h"
#include "Definitions/configurationDefinitions.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"

class HlcSingleAircraft : public HighLevelController<fixedWingCommands, fixedWingStates> {

public:
    HlcSingleAircraft(worldPhysics& physics, float rate);
    fixedWingCommands calculateInputs(const std::vector<fixedWingStates>& currentStates, const target& target) override;

private:
    worldPhysics m_physics;
    float m_rate;

    std::unique_ptr<GroundspeedController> m_groundspeedController;
};


#endif //GRS_SIMULATOR_HLCSINGLEAIRCRAFT_H
