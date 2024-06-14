/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "system.h"

System::System(worldPhysics& worldPhysics)
    : m_physics(worldPhysics)
{

}

void System::update(double dt) {
    // Call tether update()
    // Call fixedWing update()

}

void System::createSystemOneAircraft() {

}

void System::createSystemOneAircraftGrounded() {

}

void System::createSystemTwoAircraftGroundedV() {

}

void System::createSystemTwoAircraftFreeV() {

}

void System::createSystemTwoAircraftGroundedY() {

}

void System::createSystemTwoAircraftFreeY() {

}
