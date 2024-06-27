/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LLCBASEVERSION_H
#define GRS_SIMULATOR_LLCBASEVERSION_H

#include <memory>

#include "Control/lowLevelController.h"
#include "Control/pitchController.h"
#include "Control/rollController.h"
#include "Control/altitudeController.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"

class LlcBaseVersion : public LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands> {

public:
    LlcBaseVersion(worldPhysics& physics, float rate);

    fixedWingControlInputs calculateAircraftControlInputs(const fixedWingStates& currentStates, const fixedWingCommands& commands) override;

private:
    worldPhysics m_physics;

    // Controller rate Hz
    float m_rate;

    // desired[roll, pitch, yaw] --> navigation[roll, pitch, yaw]
    // desired commands are modified to account coupling and ++
    void m_calcNavigationRoll(double commandedRoll);
    void m_calcNavigationPitch(double commandedPitch);
    void m_calcNavigationYaw(double commandedYaw);

    double m_navigationRoll = 0.0;
    double m_navigationPitch = 0.0;
    double m_navigationYaw = 0.0;

    // stabilize aircraft attitude at navigation[roll, pitch, yaw]
    int32_t m_stabilizeRoll(double commandedRoll);
    int32_t m_stabilizePitch(double commandedPitch);
    int32_t m_stabilizeYaw(double commandedYaw);
    int32_t m_stabilizeRpm(double commandedRpm);

    // PixHawk controller
    std::unique_ptr<RollController> m_rollController;
    std::unique_ptr<PitchController>  m_pitchController;
    std::unique_ptr<AltitudeController> m_altController;
};


#endif //GRS_SIMULATOR_LLCBASEVERSION_H
