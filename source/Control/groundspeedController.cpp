/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "groundspeedController.h"

GroundspeedController::GroundspeedController(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

float GroundspeedController::getThrustOut(float desiredSpeed, bool disableIntegrator) {
    m_thrustPid.setDt(m_dt);

    // get groundspeed m/s
    auto airspeed = static_cast<float>(std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w));
    float groundspeed = airspeed * static_cast<float>(std::cos(m_aircraftStates.theta));

    // reset I if required
    if (disableIntegrator){
        m_thrustPid.resetI();
    }

    // compute errors
    m_thrustPid.updateAll(desiredSpeed, groundspeed, false);
    m_pidInfo = m_thrustPid.getPidInfo();

    float thrustDem = m_pidInfo.P + m_pidInfo.I + m_pidInfo.D;

    return constrainFloat(thrustDem, 0.0f, 60.0f);
}

void GroundspeedController::setAircraftStates(const fixedWingStates& aircraftStates) {
    m_aircraftStates = aircraftStates;
}
