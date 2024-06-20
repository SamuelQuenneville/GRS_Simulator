/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "system.h"

System::System(worldPhysics& worldPhysics, systemConfiguration& configuration)
    : m_physics(worldPhysics)
    , m_systemConfiguration(configuration)
{

}

void System::update(double dt) {
    // Call tether update()
    // Call fixedWing update()
    for(const auto& aircraft : m_rigidBodies) {
        aircraft->update(dt);

    }
}

void System::createSystemOneAircraft(fixedWingProperties& aircraftProperties, initialStates& aircraftStates) {
    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> llc = std::make_unique<LlcBaseVersion>(m_physics, m_systemConfiguration.llcRate);

    std::unique_ptr<RigidBody> aircraft = std::make_unique<FixedWing>(m_physics, aircraftProperties, std::move(llc));
    aircraft->setInitialStates(aircraftStates);

    m_rigidBodies.push_back(std::move(aircraft));
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
