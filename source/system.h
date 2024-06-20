/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SYSTEM_H
#define GRS_SIMULATOR_SYSTEM_H

#include "Definitions/worldPhysics.h"
#include "RigidBodyAndLink//fixedWing.h"
#include "RigidBodyAndLink/tether.h"

#include "Definitions/configurationDefinitions.h"
#include "Definitions/systemDefinitions.h"
#include "Control/llcBaseVersion.h"

class System {

public:
    explicit System(worldPhysics& worldPhysics, systemConfiguration& configuration);

    void update(double dt);

    // Create the different configuration of system
    void createSystemOneAircraft(fixedWingProperties& aircraftProperties, initialStates& aircraftStates);
    void createSystemOneAircraftGrounded();
    void createSystemTwoAircraftGroundedV();
    void createSystemTwoAircraftFreeV();
    void createSystemTwoAircraftGroundedY();
    void createSystemTwoAircraftFreeY();

private:
    worldPhysics& m_physics;
    systemConfiguration& m_systemConfiguration;

    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    std::vector<Link*> m_links;


};


#endif //GRS_SIMULATOR_SYSTEM_H
