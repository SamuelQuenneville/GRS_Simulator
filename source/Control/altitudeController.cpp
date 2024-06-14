/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "altitudeController.h"

AltitudeController::AltitudeController(worldPhysics &physics, float rate)
    : m_physics(physics)
    , _dt(1.0f/rate)
{

}

float AltitudeController::getPitchOut(float desiredAltitude, bool disableIntegrator) {
    // Limit integrator if the last pitch output too high
    bool limit_I = fabsf(_lastOutRad) >= radians(20.0);
    altPID.set_dt(_dt);

    // Get airspeed
    float airspeed = std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w);

    bool underspeed = airspeed <= 11.0;
    if (underspeed || disableIntegrator) {
        limit_I = true;
    }

    if (disableIntegrator) {
        altPID.reset_I();
    }

    // Get altitude
    float alt_m = -m_aircraftStates.z;

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    altPID.update_all(desiredAltitude, alt_m, limit_I);

    _pidInfo = altPID.getPID_info();
    auto &pinfo = _pidInfo;
    // No feedforward
    pinfo.FF = 0.0f;

    // sum components
    float pitch_out_rad = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    _lastOutRad = pitch_out_rad;

    // output is scaled to centidegrees of pitch
    return constrain_float(degrees(pitch_out_rad)*100, -3000, 3000);
}

void AltitudeController::resetIntegrator() {
    _pidInfo.I = 0;
    altPID.reset_I();
}

void AltitudeController::resetIntegratorTakeoff() {
    float iRad = radians(20.0);
    _pidInfo.I = iRad;
    altPID.set_integrator(iRad);
}
