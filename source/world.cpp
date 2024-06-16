/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"

World::World() {

    systemTypeStringToEnum = {
            {"ONE_AIRCRAFT", ONE_AIRCRAFT},
            {"ONE_AIRCRAFT_GROUNDED", ONE_AIRCRAFT_GROUNDED},
            {"TWO_AIRCRAFT_GROUNDED_V", TWO_AIRCRAFT_GROUNDED_V},
            {"TWO_AIRCRAFT_FREE_V", TWO_AIRCRAFT_FREE_V},
            {"TWO_AIRCRAFT_GROUNDED_Y", TWO_AIRCRAFT_GROUNDED_Y},
            {"TWO_AIRCRAFT_FREE_Y", TWO_AIRCRAFT_FREE_Y}
    };
}

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

int World::getNumberOfAircraftInSystem(World::systemType type) {

    if (type == ONE_AIRCRAFT || type == ONE_AIRCRAFT_GROUNDED) {
        return 1;
    } else {
        return 2;
    }
}
