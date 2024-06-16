/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "altitudeController.h"

AltitudeController::AltitudeController(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

float AltitudeController::getPitchOut(float desiredAltitude, bool disableIntegrator) {
    // Limit integrator if the last pitch output too high
    bool limitIntegrator = fabsf(m_lastOutRad) >= radians(20.0);
    m_altPID.setDt(m_dt);

    // Get airspeed
    float airspeed = std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w);

    bool underspeed = airspeed <= 11.0;
    if (underspeed || disableIntegrator) {
        limitIntegrator = true;
    }

    if (disableIntegrator) {
        m_altPID.resetI();
    }

    // Get altitude
    float altitudeMeters = -m_aircraftStates.z;

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    m_altPID.updateAll(desiredAltitude, altitudeMeters, limitIntegrator);

    m_pidInfo = m_altPID.getPidInfo();
    auto& pinfo = m_pidInfo;
    // No feedforward
    pinfo.FF = 0.0f;

    // sum components
    float pitchOutRad = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    m_lastOutRad = pitchOutRad;

    // output is scaled to centidegrees of pitch
    return constrainFloat(degrees(pitchOutRad) * 100, -3000, 3000);
}

void AltitudeController::resetIntegrator() {
    m_pidInfo.I = 0;
    m_altPID.resetI();
}

void AltitudeController::resetIntegratorTakeoff() {
    float iRad = radians(20.0);
    m_pidInfo.I = iRad;
    m_altPID.setIntegrator(iRad);
}
