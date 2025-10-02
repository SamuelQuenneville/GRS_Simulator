/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "phaseController.h"

PhaseController::PhaseController(const worldPhysics& physics, phaseControllerGains& gains, const float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
{

}

float PhaseController::getThrustOut(const float desiredPhase, const payloadStates& payloadStates) {
    // phase angle
    m_ctrlInfo.phase = std::atan2(static_cast<float>(m_aircraftStates.y - payloadStates.y), static_cast<float>(m_aircraftStates.x - payloadStates.x));
    m_ctrlInfo.phaseTarget = desiredPhase;

    // compute errors
    float error = grs::wrapToPi(m_loiterDirection * (m_ctrlInfo.phaseTarget - m_ctrlInfo.phase));
    error = grs::unwrap(m_lastError, error);

    m_ctrlInfo.P = std::clamp(error, -grs::degToRad(20.0f), grs::degToRad(20.0f)) * m_gains.P;
    m_ctrlInfo.D = m_gains.D * (error - m_lastError) / m_dt;
    m_ctrlInfo.I += (error * m_gains.I) * m_dt;
    m_ctrlInfo.I = std::clamp(m_ctrlInfo.I, -m_gains.iLimit, m_gains.iLimit);

    const float thrustDem = m_ctrlInfo.P + m_ctrlInfo.D + m_ctrlInfo.I;
    m_ctrlInfo.thrustOut = std::clamp(thrustDem, -m_gains.thrustSaturation, m_gains.thrustSaturation);

    return m_ctrlInfo.thrustOut;
}

void PhaseController::setAircraftStates(const fixedWingStates& aircraftStates) {
    m_aircraftStates = aircraftStates;
}

void PhaseController::setGains(const phaseControllerGains& gains) const {
    m_gains = gains;
}
