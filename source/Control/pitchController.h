/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PITCHCONTROLLER_H
#define GRS_SIMULATOR_PITCHCONTROLLER_H

#include "Control/pid.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"

#include "Mathematics/grsMath.h"

class PitchController {

public:
    explicit PitchController(worldPhysics &physics, float rate);

    int32_t getServoOut(float navPitch, float scaler, bool disableIntegrator);
    int32_t getRateOut(float desiredRate, float scaler, bool disableIntegrator, float airspeed);
    float getCoordinationRateOffset(float airspeed) const;

    void setAircraftStates(const fixedWingStates &aircraftStates) {_aircraftStates = aircraftStates;};

    void resetIntegrator();

private:
    worldPhysics m_physics;

    // pitch time constant
    static constexpr float _tau = 0.5;

    // maximum & minimum pitch rate
    static constexpr float _rmax = 20.0;
    static constexpr float _rmin = 20.0; // declare positive

    // roll compensation
    static constexpr float _rollCompensation = 1.0;

    // controller frequency
    float _dt;

    float _lastOut = 0.0;
    float _angleErrorDeg = 0.0;

    PID ratePID{0.04, 0.15, 0, 0.345, 0.666, 1.5, 7, 4, 0.02, 150, 1};
    PID::PID_Info _pidInfo{};

    fixedWingStates _aircraftStates{};
};

#endif //GRS_SIMULATOR_PITCHCONTROLLER_H
