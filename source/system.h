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

class System {

public:
    explicit System(worldPhysics& worldPhysics);

    void update(double dt);

    // Create the different configuration of system
    void createSystemOneAircraft();
    void createSystemOneAircraftGrounded();
    void createSystemTwoAircraftGroundedV();
    void createSystemTwoAircraftFreeV();
    void createSystemTwoAircraftGroundedY();
    void createSystemTwoAircraftFreeY();

private:
    worldPhysics& m_physics;

    std::vector<RigidBody*> m_rigidBodies;


};


#endif //GRS_SIMULATOR_SYSTEM_H
