/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "rollController.h"

RollController::RollController(worldPhysics &physics, float rate)
    : m_physics(physics)
    , _dt(1.0f/rate)
{

}

int32_t RollController::getServoOut(float navRoll, float scaler, bool disableIntegrator) {
    // Calculate the desired roll rate (deg/sec) from the angle error
    _angleErrorDeg = navRoll - degrees(_aircraftStates.phi);
    float desiredRate = _angleErrorDeg / _tau;

    // limit the demanded roll rate
    if (desiredRate < -_rmax) {
        desiredRate = - _rmax;
    } else if (desiredRate > _rmax) {
        desiredRate = _rmax;
    }

    return getRateOut(desiredRate, scaler, disableIntegrator);
}

int32_t RollController::getRateOut(float desiredRate, float scaler, bool disableIntegrator) {
    bool limitIntegrator = fabsf(_lastOut) >= 45;
    float oldIntegrator = ratePID.get_i();
    ratePID.set_dt(_dt);

    float airspeed = sqrt(_aircraftStates.u * _aircraftStates.u + _aircraftStates.v * _aircraftStates.v + _aircraftStates.w * _aircraftStates.w);
    bool underspeed = airspeed <= 11.0;
    if(underspeed){
        limitIntegrator = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    ratePID.update_all(radians(desiredRate) * scaler * scaler, _aircraftStates.p * scaler * scaler, limitIntegrator);

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
    pinfo.actual = degrees(_aircraftStates.p);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    _lastOut = out;

    // output is scaled to notional centidegrees of deflection
    return constrain_int32(out * 100, -1500, 1500);
}

void RollController::resetIntegrator() {
    _pidInfo.I = 0;
    ratePID.reset_I();
}
