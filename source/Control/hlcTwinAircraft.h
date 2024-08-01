/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_HLCTWINAIRCRAFT_H
#define GRS_SIMULATOR_HLCTWINAIRCRAFT_H

#include "Control/highLevelController.h"
#include "Control/groundspeedController.h"
#include "Control/lateralController.h"

class HlcTwinAircraft : public HighLevelController<fixedWingCommands, fixedWingStates> {

public:
    HlcTwinAircraft(worldPhysics& physics, Logger& logger, float rate);
    std::map<int, fixedWingCommands> calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) override;

private:
    worldPhysics m_physics;
    Logger& m_logger;
    float m_rate;

    std::unique_ptr<GroundspeedController> m_groundspeedController1;
    std::unique_ptr<GroundspeedController> m_groundspeedController2;
    std::unique_ptr<LateralController> m_lateralController1;
    std::unique_ptr<LateralController> m_lateralController2;

    void m_writeLog(const fixedWingCommands& commands1, const fixedWingCommands& commands2);
};


#endif //GRS_SIMULATOR_HLCTWINAIRCRAFT_H
