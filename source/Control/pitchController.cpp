/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "pitchController.h"


PitchController::PitchController(worldPhysics& physics, pitchControllerGains& gains, float rate)
    : m_physics(physics)
    , m_gains(gains)
    , m_dt(1.0f/rate)
{
    m_ratePID(m_gains.P, m_gains.I, m_gains.D, m_gains.FF, -m_gains.iLimit, m_gains.iLimit,
              m_gains.targetFilter, m_gains.errorFilter, m_gains.derivativeFilter, m_dt);
}

float PitchController::getRateOut(float desiredRate, float scaler, bool disableIntegrator, float airspeed) {
    bool limitIntegrator = std::fabs(m_lastOut) >= 45.0f;
    float oldIntegrator = m_ratePID.getI();
    m_ratePID.setDt(m_dt);

    bool underspeed = airspeed <= 11.0f;
    if(underspeed) {
        limitIntegrator = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    m_ratePID.updateAll(radians(desiredRate) * scaler * scaler, static_cast<float>(m_aircraftStates.q) * scaler * scaler, limitIntegrator);

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
    m_pidInfo.actual = degrees(static_cast<float>(m_aircraftStates.q));

    // sum components
    m_lastOut = m_pidInfo.P + m_pidInfo.I + m_pidInfo.D + m_pidInfo.FF;

    // output is scaled to notional centidegrees of deflection
    return constrainFloat(m_lastOut, m_gains.elevatorMin, m_gains.elevatorMax);
}

float PitchController::getCoordinationRateOffset(float airspeed) const {
    float rateOffset;
    auto bankAngle = static_cast<float>(m_aircraftStates.phi);

    // limit bank angle between +- 80 deg if right way up
    if (std::fabs(bankAngle) < (M_PI/2.0))	{
        bankAngle = constrainFloat(bankAngle, -radians(80), radians(80));
    } else {
        if (bankAngle > 0.0f) {
            bankAngle = constrainFloat(bankAngle, radians(100), M_PI);
        } else {
            bankAngle = constrainFloat(bankAngle, -M_PI, -radians(100));
        }
    }

    if (std::fabs(degrees(m_aircraftStates.theta)) > 70.0) {
        // don't do turn coordination handling when at very high pitch angles
        rateOffset = 0.0f;
    } else {
        rateOffset = std::cos(static_cast<float>(m_aircraftStates.theta))*std::fabs(degrees((static_cast<float>(m_physics.gravity / std::fmax(airspeed , 11.0f)) * std::tan(bankAngle) * std::sin(bankAngle)))) * m_rollCompensation;
    }

    return rateOffset;
}

float PitchController::getServoOut(float navPitch, float scaler, bool disableIntegrator) {
    auto airspeed = static_cast<float>(std::sqrt(m_aircraftStates.u * m_aircraftStates.u + m_aircraftStates.v * m_aircraftStates.v + m_aircraftStates.w * m_aircraftStates.w));

    float rateOffset = getCoordinationRateOffset(airspeed);

    // Calculate the desired pitch rate (deg/sec) from the angle error
    m_angleErrorDeg = navPitch - degrees(static_cast<float>(m_aircraftStates.theta));
    float desiredRate = m_angleErrorDeg / m_tau;

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
    m_pidInfo.I = 0.0f;
    m_ratePID.resetI();
}
