/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LATERALCONTROLLER_H
#define GRS_SIMULATOR_LATERALCONTROLLER_H

#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/worldPhysics.h"
#include "Definitions/controllerGains.h"

class LateralController {

public:
    explicit LateralController(const worldPhysics& physics, lateralControllerGains& gains, float rate);

    void updateLoiter(const grs::Vec2d& centerWP, float radius, float loiterDirection, bool isTethered);
    [[nodiscard]] float navRollCmd() const;

    void setAircraftStates(const fixedWingStates& aircraftStates) {m_aircraftStates = aircraftStates;};

    struct ctrlInfo {
        double  actualPositionX;
        double  targetPositionX;
        double  actualPositionY;
        double  targetPositionY;
        float   radius;
        float   kx;
        float   xtrackPositionError;
        float   kv;
        float   xtrackVelocityError;
        float   accelerationTarget;
    };

    [[nodiscard]] const ctrlInfo& getCtrlInfo() const {return m_ctrlInfo;};

private:
    worldPhysics m_physics;
    fixedWingStates m_aircraftStates{};
    lateralControllerGains& m_gains;

    ctrlInfo m_ctrlInfo{};

    float m_dt;

    float m_integrator = 0.0f;

    float m_periodL1;
    float m_dampingL1;

    // lateral acceleration in m/s require for L1 guidance
    float m_latAccDem = 0.0f;
    float m_distanceL1 = 0.0f;
};


#endif //GRS_SIMULATOR_LATERALCONTROLLER_H
