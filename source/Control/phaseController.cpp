/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "phaseController.h"

PhaseController::PhaseController(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

float PhaseController::getThrustOut(float desiredPhase, bool disableIntegrator) {
    m_phasePid.setDt(m_dt);

    // reset I if required
    if (disableIntegrator){
        m_phasePid.resetI();
    }

    // phase angle
    auto phase = static_cast<float>(std::atan2(m_aircraftStates.y, m_aircraftStates.x));

    // compute errors
    float error = wrapPI(-1.0f*(desiredPhase-phase));
    error = unwrap(m_lastError, error);

    float proportionalOut = constrainFloat(error, -radians(20.0f), radians(20.0f)) * 50.0f;
    float derivative = 0.5f * (error - m_lastError) / m_dt;
    m_integrator += (error * 8.0f) * m_dt;
    m_integrator = constrainFloat(m_integrator, -15.0f, 15.0f);

    if(error > radians(90.0f)){
        m_integrator = 0.0f;
    }

    float thrustDem = proportionalOut + derivative + m_integrator;
    return constrainFloat(thrustDem, -60.0f, 60.0f);
}

void PhaseController::setAircraftStates(const fixedWingStates& aircraftStates) {
    m_aircraftStates = aircraftStates;
}
