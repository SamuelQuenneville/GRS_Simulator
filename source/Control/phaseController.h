/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PHASECONTROLLER_H
#define GRS_SIMULATOR_PHASECONTROLLER_H

#include "Control/pid.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"
#include "Definitions/controllerGains.h"
#include "Definitions/controllerGains.h"
#include "Definitions/payloadDefinitions.h"

class PhaseController {

public:
    explicit PhaseController(worldPhysics& physics, phaseControllerGains& gains, float rate);

    float getThrustOut(float desiredPhase, const payloadStates& payloadStates);
    void setAircraftStates(const fixedWingStates& aircraftStates);

    void setGains(const phaseControllerGains& gains);

    struct ctrlInfo {
        float phase;
        float phaseTarget;
        float P;
        float I;
        float D;
        float thrustOut;
    };

    [[nodiscard]] const ctrlInfo& getCtrlInfo() const {return m_ctrlInfo;};

private:
    worldPhysics m_physics;
    phaseControllerGains& m_gains;
    ctrlInfo m_ctrlInfo{};

    float m_loiterDirection = -1.0f;

    float m_dt;
    float m_lastError = 0.0f;

    fixedWingStates m_aircraftStates{};
};


#endif //GRS_SIMULATOR_PHASECONTROLLER_H
