/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SYSTEM_H
#define GRS_SIMULATOR_SYSTEM_H

#include "RigidBodyAndLink//fixedWing.h"
#include "RigidBodyAndLink/tether.h"

#include "Definitions/systemDefinitions.h"
#include "Control/llcBaseVersion.h"
#include "Control/hlcSingleAircraft.h"


class System {

public:
    explicit System(worldPhysics& worldPhysics, Logger& logger, systemConfiguration& configuration);

    void update(double dt);
    void setTarget(const target& newTarget) {m_target = newTarget;};

    // Create the different configuration of system
    void createSystemOneAircraft(fixedWingProperties& aircraftProperties, initialStates& aircraftStates);
    void createSystemOneAircraftGrounded(fixedWingProperties &aircraftProperties, tetherProperties &tetherProperties, initialStates &aircraftStates);
    void createSystemTwoAircraftGroundedV();
    void createSystemTwoAircraftFreeV();
    void createSystemTwoAircraftGroundedY();
    void createSystemTwoAircraftFreeY();

    std::vector<std::unique_ptr<FixedWing>>& aircraft() {return m_aircraft;};

private:
    worldPhysics& m_physics;
    Logger& m_logger;
    systemConfiguration& m_systemConfiguration;

    double m_timeSinceHlcCall;
    bool m_shouldCallHlc();

    void m_initHlcCommands();

    target m_target{};
    fixedWingCommands m_commands{};
    std::unique_ptr<HlcSingleAircraft> m_controller;

    std::vector<fixedWingStates> m_getAircraftStates();

    std::vector<std::unique_ptr<FixedWing>> m_aircraft;
    std::vector<std::unique_ptr<Tether>> m_links;
};


#endif //GRS_SIMULATOR_SYSTEM_H
