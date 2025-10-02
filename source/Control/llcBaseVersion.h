/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LLCBASEVERSION_H
#define GRS_SIMULATOR_LLCBASEVERSION_H

#include "Control/lowLevelController.h"
#include "Control/pitchController.h"
#include "Control/rollController.h"
#include "Control/altitudeController.h"

class LlcBaseVersion final : public LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands> {

public:
    LlcBaseVersion(const worldPhysics& physics, Logger& logger, llcGains& gains, float rate);

    fixedWingControlInputs calculateAircraftControlInputs(int aircraftId, const fixedWingStates& currentStates, const fixedWingCommands& commands) override;

private:
    worldPhysics m_physics;
    Logger& m_logger;
    llcGains& m_gains;

    // Controller rate Hz
    float m_rate;

    // desired[roll, pitch, yaw] --> navigation[roll, pitch, yaw]
    // desired commands are modified to account coupling and ++
    void m_calcNavigationRoll(double commandedRoll);
    void m_calcNavigationPitch(double commandedPitch);
    void m_calcNavigationYaw(double commandedYaw);

    float m_navigationRoll = 0.0;
    float m_navigationPitch = 0.0;
    float m_navigationYaw = 0.0;

    // stabilize aircraft attitude at navigation[roll, pitch, yaw]
    float m_stabilizeRoll(double commandedRoll);
    float m_stabilizePitch(double commandedPitch);
    int32_t m_stabilizeYaw(double commandedYaw);
    int32_t m_stabilizeRpm(double commandedRpm);

    // PixHawk controller
    std::unique_ptr<RollController> m_rollController;
    std::unique_ptr<PitchController>  m_pitchController;
    std::unique_ptr<AltitudeController> m_altController;

    void m_writeLog(int aircraftId, const fixedWingControlInputs& inputs) const;
};


#endif //GRS_SIMULATOR_LLCBASEVERSION_H
