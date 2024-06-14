/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"

World::World() = default;

void World::update() {

}

void World::log() {

}

void World::createSystem(systemType type) {

    m_system = std::make_unique<System>(worldPhysics);

    switch(type) {
        case ONE_AIRCRAFT:
            m_system->createSystemOneAircraft();
            break;
        case ONE_AIRCRAFT_GROUNDED:
            m_system->createSystemOneAircraftGrounded();
            break;
        case TWO_AIRCRAFT_GROUNDED_V:
            m_system->createSystemTwoAircraftGroundedV();
            break;
        case TWO_AIRCRAFT_FREE_V:
            m_system->createSystemTwoAircraftFreeV();
            break;
        case TWO_AIRCRAFT_GROUNDED_Y:
            m_system->createSystemTwoAircraftGroundedY();
            break;
        case TWO_AIRCRAFT_FREE_Y:
            m_system->createSystemTwoAircraftFreeY();
            break;
    }
}