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

void World::readConfigurationFile(const std::string& fileName) {
    YAML::Node nodeConfiguration = YAML::LoadFile(fileName);
    simulationParameters simParams = ConfigurationParser::parseSimulationParameters(nodeConfiguration);

    m_simulationConfiguration = {simParams.fixedDuration, simParams.duration};
    m_systemConfiguration = {systemTypeStringToEnum.at(simParams.systemConfiguration), simParams.hlcRate, simParams.llcRate, simParams.windOn, simParams.tetherAeroOn};

    m_aircraftProperties = ConfigurationParser::parseFixedWingProperties(simParams.aircraftPropertiesFileDirectory);
    m_tetherProperties = ConfigurationParser::parseTetherProperties(simParams.tetherPropertiesFileDirectory);

    int numberAircraftInSystem = getNumberOfAircraftInSystem(m_systemConfiguration.type);

    for (int i = 1; i <= numberAircraftInSystem; i++) {
        std::string nodeName = "InitialStatesAircraft_" + std::to_string(i);
        initialStates states = ConfigurationParser::parseInitialStates(nodeConfiguration, nodeName);
        m_initialAircraftStatesVector.push_back(&states);
    }

    m_initialPayloadStates = ConfigurationParser::parseInitialStates(nodeConfiguration, "InitialStatesPayload");
    m_systemTarget = ConfigurationParser::parseSystemTarget(nodeConfiguration);
}

void World::log() {

}

void World::createSystem() {

    systemType type = m_systemConfiguration.type;
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
