/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GROUNDSPEEDCONTROLLER_H
#define GRS_SIMULATOR_GROUNDSPEEDCONTROLLER_H

#include "Control/pid.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"

class GroundspeedController {

public:
    explicit GroundspeedController(worldPhysics& physics, float rate);

    float getThrustOut(float desiredSpeed, bool disableIntegrator);
    void setAircraftStates(const fixedWingStates& aircraftStates);

    const PID::pidInfo& getPidInfo() const { return m_pidInfo;};

private:
    worldPhysics m_physics;

    float m_dt;

    PID m_thrustPid{2.0,2.0,0.5,0.0,20.0,3.0,10.0,3.0,0.02,360.0,1.0};
    PID::pidInfo m_pidInfo{};

    fixedWingStates m_aircraftStates{};
};


#endif //GRS_SIMULATOR_GROUNDSPEEDCONTROLLER_H
