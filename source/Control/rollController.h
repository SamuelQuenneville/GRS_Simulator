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

class RollController {

public:
    explicit RollController(worldPhysics& physics, float rate);

    int32_t getServoOut(float navRoll, float scaler, bool disableIntegrator);
    int32_t getRateOut(float desiredRate, float scaler, bool disableIntegrator);

    [[nodiscard]] const PID::pidInfo& getPidInfo() const { return m_pidInfo;};

    void setAircraftStates(const fixedWingStates& aircraftStates) {m_aircraftStates = aircraftStates;};

    void resetIntegrator();

private:
    worldPhysics m_physics;

    // roll time constant
    static constexpr float m_tau = 0.5f;

    // maximum roll rate (degrees/sec)
    static constexpr float m_rmax = 60.0f;

    // controller frequency
    float m_dt;

    float m_lastOut = 0.0f;
    float m_angleErrorDeg = 0.0f;

    PID m_ratePID{0.08, 0.15, 0, 0.345, 0.666, 3, 7, 4, 0.02, 150, 1};
    PID::pidInfo m_pidInfo{};

    fixedWingStates m_aircraftStates{};
};


#endif //GRS_SIMULATOR_ROLLCONTROLLER_H
