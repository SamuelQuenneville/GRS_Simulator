/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_ROLLCONTROLLER_H
#define GRS_SIMULATOR_ROLLCONTROLLER_H

#include "Control/pid.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"

#include "Mathematics/grsMath.h"

class RollController {

public:
    explicit RollController(worldPhysics &physics, float rate);

    int32_t getServoOut(float navRoll, float scaler, bool disableIntegrator);
    int32_t getRateOut(float desiredRate, float scaler, bool disableIntegrator);

    void setAircraftStates(const fixedWingStates &aircraftStates) {_aircraftStates = aircraftStates;};

    void resetIntegrator();

private:
    worldPhysics m_physics;

    // roll time constant
    static constexpr float _tau = 0.5;

    // maximum roll rate (degrees/sec)
    static constexpr float _rmax = 60.0;

    // controller frequency
    float _dt;

    float _lastOut = 0.0;
    float _angleErrorDeg = 0.0;

    PID ratePID{0.08, 0.15, 0, 0.345, 0.666, 3, 7, 4, 0.02, 150, 1};
    PID::PID_Info _pidInfo{};

    fixedWingStates _aircraftStates{};

};


#endif //GRS_SIMULATOR_ROLLCONTROLLER_H
