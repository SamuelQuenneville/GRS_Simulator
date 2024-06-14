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
    , _dt(1.0f/rate)
{

}

int32_t PitchController::getRateOut(float desiredRate, float scaler, bool disableIntegrator, float airspeed)
{
    bool limitIntegrator = fabsf(_lastOut) >= 45;
    float oldIntegrator = ratePID.get_i();
    ratePID.set_dt(_dt);

    bool underspeed = airspeed <= 11.0;
    if(underspeed){
        limitIntegrator = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    ratePID.update_all(radians(desiredRate) * scaler * scaler, _aircraftStates.q * scaler * scaler, limitIntegrator);

    if (underspeed) {
        // when underspeed we lock the integrator
        ratePID.set_integrator(oldIntegrator);
    }

    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(ratePID.get_ff() / scaler);

    if (disableIntegrator) {
        ratePID.reset_I();
    }

    // convert AC_PID info object to same scale as old controller
    _pidInfo = ratePID.getPID_info();
    auto &pinfo = _pidInfo;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desiredRate;
    pinfo.actual = degrees(_aircraftStates.q);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;
    // remember the last output to trigger the I limit
    _lastOut = out;

    // output is scaled to notional centidegrees of deflection
    return constrain_int32(out * 100, -2000, 2000);
}

float PitchController::getCoordinationRateOffset(float airspeed) const
{
    float rateOffset;
    float bankAngle = _aircraftStates.phi;

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bankAngle) < radians(90))	{
        bankAngle = constrain_float(bankAngle,-radians(80),radians(80));
    } else {
        if (bankAngle > 0.0f) {
            bankAngle = constrain_float(bankAngle,radians(100),radians(180));
        } else {
            bankAngle = constrain_float(bankAngle,-radians(180),-radians(100));
        }
    }

    if (std::abs(degrees(_aircraftStates.theta)) > 70.0) {
        // don't do turn coordination handling when at very high pitch angles
        rateOffset = 0.0;
    } else {
        rateOffset = cosf(_aircraftStates.theta)*fabsf(degrees((m_physics.gravity / MAX(airspeed , 11.0)) * tanf(bankAngle) * sinf(bankAngle))) * _rollCompensation;
    }

    return rateOffset;
}

int32_t PitchController::getServoOut(float navPitch, float scaler, bool disableIntegrator)
{
    float airspeed = std::sqrt(_aircraftStates.u * _aircraftStates.u + _aircraftStates.v * _aircraftStates.v + _aircraftStates.w * _aircraftStates.w);

    float rateOffset = getCoordinationRateOffset(airspeed);

    // Calculate the desired pitch rate (deg/sec) from the angle error
    _angleErrorDeg = navPitch - degrees(_aircraftStates.theta);
    float desiredRate = _angleErrorDeg / _tau;

    //qDebug() << "_angleErrorDeg = " << _angleErrorDeg << "| desiredRate = " << desiredRate;
    // limit the maximum pitch rate demand
    if (desiredRate < -_rmin) {
        desiredRate = -_rmin;
    } else if (desiredRate > _rmax) {
        desiredRate = _rmax;
    }

    // Apply the turn correction offset
    desiredRate = desiredRate + rateOffset;

    return getRateOut(desiredRate, scaler, disableIntegrator, airspeed);
}

void PitchController::resetIntegrator()
{
    _pidInfo.I = 0;
    ratePID.reset_I();
}
