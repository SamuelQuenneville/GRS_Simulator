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

class HlcSingleAircraft final : public HighLevelController<fixedWingCommands, fixedWingStates> {

public:
    HlcSingleAircraft(const worldPhysics& physics, hlcGains& gains, float rate);
    std::map<int, fixedWingCommands> calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) override;
    std::map<int, fixedWingCommands> calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const payloadStates& payloadStates, const target& target) override;


private:
    worldPhysics m_physics;
    hlcGains& m_gains;
    float m_rate;

    std::unique_ptr<GroundspeedController> m_groundspeedController;

    void m_writeLog(const fixedWingCommands& commands) const;
};


#endif //GRS_SIMULATOR_HLCSINGLEAIRCRAFT_H
