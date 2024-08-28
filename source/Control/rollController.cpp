/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rollController.h"

RollController::RollController(worldPhysics& physics, rollControllerGains& gains, float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
{
    m_ratePID(m_gains.P, m_gains.I, m_gains.D, m_gains.FF, -m_gains.iLimit, m_gains.iLimit,
              m_gains.targetFilter, m_gains.errorFilter, m_gains.derivativeFilter, m_dt);
}

float RollController::getServoOut(float navRoll, float scaler, bool disableIntegrator) {
    // Calculate the desired roll rate (deg/sec) from the angle error
    m_angleErrorDeg = navRoll - degrees(static_cast<float>(m_aircraftStates.phi));
    float desiredRate = m_angleErrorDeg / m_tau;

    // limit the demanded roll rate
    if (desiredRate < -m_rmax) {
        desiredRate = - m_rmax;
    } else if (desiredRate > m_rmax) {
        desiredRate = m_rmax;
    }

    return getRateOut(desiredRate, scaler, disableIntegrator);
}

float RollController::getRateOut(float desiredRate, float scaler, bool disableIntegrator) {
    bool limitIntegrator = std::fabs(m_lastOut) >= 45.0f;
    float oldIntegrator = m_ratePID.getI();
    m_ratePID.setDt(m_dt);

    auto airspeed = static_cast<float>(std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w));
    bool underspeed = airspeed <= 11.0f;
    if(underspeed) {
        limitIntegrator = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    m_ratePID.updateAll(radians(desiredRate) * scaler * scaler, static_cast<float>(m_aircraftStates.p) * scaler * scaler, limitIntegrator);

    if (underspeed) {
        // when underspeed we lock the integrator
        m_ratePID.setIntegrator(oldIntegrator);
    }

    if (disableIntegrator) {
        m_ratePID.resetI();
    }

    // convert AC_PID info object to same scale as old controller
    m_pidInfo = m_ratePID.getPidInfo();

    m_pidInfo.P  = degrees(m_pidInfo.P);
    m_pidInfo.I  = degrees(m_pidInfo.I);
    m_pidInfo.D  = degrees(m_pidInfo.D);

    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    m_pidInfo.FF = degrees(m_ratePID.getFF() / scaler);

    // fix the logged target and actual values to not have the scalers applied
    m_pidInfo.target = desiredRate;
    m_pidInfo.actual = degrees(static_cast<float>(m_aircraftStates.p));

    // sum components
    m_lastOut = m_pidInfo.P + m_pidInfo.I + m_pidInfo.D + m_pidInfo.FF;

    // output is scaled to notional centidegrees of deflection
    return constrainFloat(m_lastOut, m_gains.aileronMin,  m_gains.aileronMax);
}

void RollController::resetIntegrator() {
    m_pidInfo.I = 0.0f;
    m_ratePID.resetI();
}
