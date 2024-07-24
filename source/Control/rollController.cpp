/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rollController.h"

RollController::RollController(worldPhysics& physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

int32_t RollController::getServoOut(float navRoll, float scaler, bool disableIntegrator) {
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

int32_t RollController::getRateOut(float desiredRate, float scaler, bool disableIntegrator) {
    bool limitIntegrator = fabsf(m_lastOut) >= 45.0f;
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

    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(m_ratePID.getFF() / scaler);

    if (disableIntegrator) {
        m_ratePID.resetI();
    }

    // convert AC_PID info object to same scale as old controller
    m_pidInfo = m_ratePID.getPidInfo();
    auto& pinfo = m_pidInfo;

    const float degScale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= degScale;
    pinfo.I *= degScale;
    pinfo.D *= degScale;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desiredRate;
    pinfo.actual = degrees(static_cast<float>(m_aircraftStates.p));

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    m_lastOut = out;

    // output is scaled to notional centidegrees of deflection
    return constrainInt32(static_cast<int32_t>(out) * 100, -1500, 1500);
}

void RollController::resetIntegrator() {
    m_pidInfo.I = 0.0f;
    m_ratePID.resetI();
}
