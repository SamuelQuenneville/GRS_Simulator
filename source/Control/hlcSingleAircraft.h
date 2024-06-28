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

class HlcSingleAircraft : public HighLevelController<fixedWingCommands, fixedWingStates> {

public:
    HlcSingleAircraft(worldPhysics& physics, Logger& logger, float rate);
    fixedWingCommands calculateInputs(const std::vector<fixedWingStates>& currentStates, const target& target) override;

private:
    worldPhysics m_physics;
    Logger& m_logger;
    float m_rate;

    std::unique_ptr<GroundspeedController> m_groundspeedController;

    void m_writeLog(const fixedWingCommands& commands);
};


#endif //GRS_SIMULATOR_HLCSINGLEAIRCRAFT_H
