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

class PhaseController {

public:
    explicit PhaseController(worldPhysics& physics, float rate);

    float getThrustOut(float desiredPhase, bool disableIntegrator);
    void setAircraftStates(const fixedWingStates& aircraftStates);

    [[nodiscard]] const PID::pidInfo& getPidInfo() const { return m_pidInfo;};
private:
    worldPhysics m_physics;

    float m_dt;
    float m_integrator=0.0f;
    float m_lastError=0.0f;

    PID m_phasePid{20.0,8.0,0.0,0.0,50.0,3.0,10.0,3.0,0.02,360.0,1.0};
    PID::pidInfo m_pidInfo{};

    fixedWingStates m_aircraftStates{};
};


#endif //GRS_SIMULATOR_PHASECONTROLLER_H
