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
#include "Control/phaseController.h"

class HlcTwinAircraft final : public HighLevelController<fixedWingCommands, fixedWingStates> {

public:
    HlcTwinAircraft(const worldPhysics& physics, Logger& logger, hlcGains& gains, float rate, double tetherLength);
    std::map<int, fixedWingCommands> calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const target& target) override;
    std::map<int, fixedWingCommands> calculateInputs(const std::vector<int>& ids, const std::vector<fixedWingStates>& currentStates, const payloadStates& payloadStates, const target& target) override;

private:
    worldPhysics m_physics;
    Logger& m_logger;
    hlcGains& m_gains;
    float m_rate;

    double m_tetherLength;

    float m_phaseDesired1 = 0.0;
    float m_phaseDesired2 = 0.0;

    std::unique_ptr<GroundspeedController> m_groundspeedController1;
    std::unique_ptr<GroundspeedController> m_groundspeedController2;
    std::unique_ptr<LateralController> m_lateralController1;
    std::unique_ptr<LateralController> m_lateralController2;
    std::unique_ptr<PhaseController> m_phaseController1;
    std::unique_ptr<PhaseController> m_phaseController2;

    void m_writeLog(const fixedWingCommands& commands1, const fixedWingCommands& commands2) const;
};


#endif //GRS_SIMULATOR_HLCTWINAIRCRAFT_H
