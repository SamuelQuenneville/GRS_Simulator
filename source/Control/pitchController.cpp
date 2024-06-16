/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "pitchController.h"


PitchController::PitchController(worldPhysics &physics, float rate)
    : m_physics(physics)
    , m_dt(1.0f/rate)
{

}

int32_t PitchController::getRateOut(float desiredRate, float scaler, bool disableIntegrator, float airspeed) {
    bool limitIntegrator = fabsf(m_lastOut) >= 45;
    float oldIntegrator = m_ratePID.getI();
    m_ratePID.setDt(m_dt);

    bool underspeed = airspeed <= 11.0;
    if(underspeed) {
        limitIntegrator = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    m_ratePID.updateAll(radians(desiredRate) * scaler * scaler, m_aircraftStates.q * scaler * scaler, limitIntegrator);

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
    pinfo.actual = degrees(m_aircraftStates.q);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;
    // remember the last output to trigger the I limit
    m_lastOut = out;

    // output is scaled to notional centidegrees of deflection
    return constrainInt32(out * 100, -2000, 2000);
}

float PitchController::getCoordinationRateOffset(float airspeed) const {
    float rateOffset;
    float bankAngle = m_aircraftStates.phi;

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bankAngle) < radians(90))	{
        bankAngle = constrainFloat(bankAngle,-radians(80),radians(80));
    } else {
        if (bankAngle > 0.0f) {
            bankAngle = constrainFloat(bankAngle,radians(100),radians(180));
        } else {
            bankAngle = constrainFloat(bankAngle,-radians(180),-radians(100));
        }
    }

    if (std::abs(degrees(m_aircraftStates.theta)) > 70.0) {
        // don't do turn coordination handling when at very high pitch angles
        rateOffset = 0.0;
    } else {
        rateOffset = cosf(m_aircraftStates.theta)*fabsf(degrees((m_physics.gravity / MAX(airspeed , 11.0)) * tanf(bankAngle) * sinf(bankAngle))) * m_rollCompensation;
    }

    return rateOffset;
}

int32_t PitchController::getServoOut(float navPitch, float scaler, bool disableIntegrator) {
    float airspeed = std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w);

    float rateOffset = getCoordinationRateOffset(airspeed);

    // Calculate the desired pitch rate (deg/sec) from the angle error
    m_angleErrorDeg = navPitch - degrees(m_aircraftStates.theta);
    float desiredRate = m_angleErrorDeg / m_tau;

    //qDebug() << "_angleErrorDeg = " << _angleErrorDeg << "| desiredRate = " << desiredRate;
    // limit the maximum pitch rate demand
    if (desiredRate < -m_rmin) {
        desiredRate = -m_rmin;
    } else if (desiredRate > m_rmax) {
        desiredRate = m_rmax;
    }

    // Apply the turn correction offset
    desiredRate = desiredRate + rateOffset;

    return getRateOut(desiredRate, scaler, disableIntegrator, airspeed);
}

void PitchController::resetIntegrator() {
    m_pidInfo.I = 0;
    m_ratePID.resetI();
}
