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

class PitchController {

public:
    explicit PitchController(worldPhysics &physics, float rate);

    int32_t getServoOut(float navPitch, float scaler, bool disableIntegrator);
    int32_t getRateOut(float desiredRate, float scaler, bool disableIntegrator, float airspeed);
    float getCoordinationRateOffset(float airspeed) const;

    const PID::pidInfo& getPidInfo() const { return m_pidInfo;};

    void setAircraftStates(const fixedWingStates &aircraftStates) {m_aircraftStates = aircraftStates;};

    void resetIntegrator();

private:
    worldPhysics m_physics;

    // pitch time constant
    static constexpr float m_tau = 0.5;

    // maximum & minimum pitch rate
    static constexpr float m_rmax = 20.0;
    static constexpr float m_rmin = 20.0; // declare positive

    // roll compensation
    static constexpr float m_rollCompensation = 1.0;

    // controller frequency
    float m_dt;

    float m_lastOut = 0.0;
    float m_angleErrorDeg = 0.0;

    PID m_ratePID{0.04, 0.15, 0, 0.345, 0.666, 1.5, 7, 4, 0.02, 150, 1};
    PID::pidInfo m_pidInfo{};

    fixedWingStates m_aircraftStates{};
};

#endif //GRS_SIMULATOR_PITCHCONTROLLER_H
