/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_ALTITUDECONTROLLER_H
#define GRS_SIMULATOR_ALTITUDECONTROLLER_H

#include "Control/pid.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"
#include "Definitions/controllerGains.h"

class AltitudeController {

public:
    explicit AltitudeController(worldPhysics& physics, altitudeControllerGains& gains, float rate);

    float getPitchOut(float desiredAltitude, bool disableIntegrator);

    void setAircraftStates(const fixedWingStates& aircraftStates) {m_aircraftStates = aircraftStates;};
    void resetIntegrator();
    void resetIntegratorTakeoff();

    [[nodiscard]] const PID::pidInfo& getPidInfo() const { return m_pidInfo;};

private:
    worldPhysics m_physics;
    altitudeControllerGains& m_gains;

    float m_dt;
    float m_lastOutRad = 0;

    PID m_altPID{radians(4.0),radians(2.0),radians(0.5),0.0,radians(20.0),3.0,10.0,3.0,0.0025,0.2,1.0};
    PID::pidInfo m_pidInfo{};

    fixedWingStates m_aircraftStates{};
};


#endif //GRS_SIMULATOR_ALTITUDECONTROLLER_H
