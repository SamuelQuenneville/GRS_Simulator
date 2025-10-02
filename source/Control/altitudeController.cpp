/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "altitudeController.h"

AltitudeController::AltitudeController(worldPhysics& physics, altitudeControllerGains& gains, float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
{
    m_altPID(m_gains.P, m_gains.I, m_gains.D, m_gains.FF, -m_gains.iLimit, m_gains.iLimit,
             m_gains.targetFilter, m_gains.errorFilter, m_gains.derivativeFilter, m_dt);
}

float AltitudeController::getPitchOut(const float desiredAltitude, const bool disableIntegrator) {
    // Limit integrator if the last pitch output too high
    bool limitIntegrator = std::fabs(m_lastOutRad) >= m_gains.iLimit;
    m_altPID.setDt(m_dt);

    // Get airspeed
    const auto airspeed = static_cast<float>(std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w));

    const bool underspeed = airspeed <= 11.0;
    if (underspeed || disableIntegrator) {
        limitIntegrator = true;
    }

    if (disableIntegrator) {
        m_altPID.resetI();
    }

    m_altPID.updateAll(desiredAltitude, static_cast<float>(-m_aircraftStates.z), limitIntegrator);
    m_pidInfo = m_altPID.getPidInfo();

    // sum components
    m_lastOutRad = m_pidInfo.P + m_pidInfo.I + m_pidInfo.D;

    // output is scaled to centidegrees of pitch
    return std::clamp(grs::radToDeg(m_lastOutRad), m_gains.pitchMin, m_gains.pitchMax);
}

void AltitudeController::resetIntegrator() {
    m_pidInfo.I = 0.0f;
    m_altPID.resetI();
}

void AltitudeController::resetIntegratorTakeoff() {
    const float iRad = m_gains.iLimit;
    m_pidInfo.I = iRad;
    m_altPID.setIntegrator(iRad);
}
