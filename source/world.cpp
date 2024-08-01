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
    m_system->update(simulationConfiguration.dt.count());
    logger().loggerClockTick(simulationConfiguration.dt.count());
}

void World::readConfigurationFile(const std::string& fileName) {
    YAML::Node nodeConfiguration = YAML::LoadFile(fileName);
    simulationParameters simParams = ConfigurationParser::parseSimulationParameters(nodeConfiguration);

    m_logDirectory = simParams.logFilesDirectory;
    m_logger = std::make_unique<Logger>(m_logDirectory);

    std::chrono::duration<double> timeStep(1.0 / simParams.llcRate);
    simulationConfiguration = {simParams.fixedDuration, simParams.duration, timeStep};
    m_systemConfiguration = {systemTypeStringToEnum.at(simParams.systemConfiguration), simParams.hlcRate, simParams.llcRate, simParams.windOn, simParams.tetherAeroOn};

    m_aircraftProperties = ConfigurationParser::parseFixedWingProperties(simParams.aircraftPropertiesFileDirectory);
    m_tetherProperties = ConfigurationParser::parseTetherProperties(simParams.tetherPropertiesFileDirectory);
    m_payloadProperties = ConfigurationParser::parsePayloadProperties(simParams.payloadPropertiesFileDirectory);

    int numberAircraftInSystem = getNumberOfAircraftInSystem(m_systemConfiguration.type);

    for (int i = 1; i <= numberAircraftInSystem; i++) {
        std::string nodeName = "InitialStatesAircraft_" + std::to_string(i);
        initialStates states = ConfigurationParser::parseInitialStates(nodeConfiguration, nodeName);
        m_initialAircraftStatesVector.push_back(states);
    }

    m_initialPayloadStates = ConfigurationParser::parseInitialStates(nodeConfiguration, "InitialStatesPayload");
    m_systemTarget = ConfigurationParser::parseSystemTarget(nodeConfiguration);
}

Logger& World::logger() {
    return *m_logger;
}

void World::createSystem() {

    m_system = std::make_unique<System>(worldPhysics, logger(), m_systemConfiguration);
    m_system->setTarget(m_systemTarget);

    systemType type = m_systemConfiguration.type;
    switch(type) {
        case ONE_AIRCRAFT:
            m_system->createSystemOneAircraft(m_aircraftProperties, m_initialAircraftStatesVector.front());
            break;
        case ONE_AIRCRAFT_GROUNDED:
            m_system->createSystemOneAircraftGrounded(m_aircraftProperties, m_tetherProperties, m_initialAircraftStatesVector.front());
            break;
        case TWO_AIRCRAFT_GROUNDED_V:
            m_system->createSystemTwoAircraftV(true, m_aircraftProperties, m_tetherProperties, m_payloadProperties, m_initialAircraftStatesVector, m_initialPayloadStates);
            break;
        case TWO_AIRCRAFT_FREE_V:
            m_system->createSystemTwoAircraftV(false, m_aircraftProperties, m_tetherProperties, m_payloadProperties, m_initialAircraftStatesVector, m_initialPayloadStates);
            break;
        case TWO_AIRCRAFT_GROUNDED_Y:
            m_system->createSystemTwoAircraftY();
            break;
        case TWO_AIRCRAFT_FREE_Y:
            m_system->createSystemTwoAircraftY();
            break;
    }
}

int World::getNumberOfAircraftInSystem(systemType type) {

    if (type == ONE_AIRCRAFT || type == ONE_AIRCRAFT_GROUNDED) {
        return 1;
    } else {
        return 2;
    }
}

systemType World::getSystemType() const {
    return m_systemConfiguration.type;
}

System* World::system() {
    return m_system.get();
}

std::string World::getExecutablePath() {
    WCHAR pathChar[MAX_PATH];
    GetModuleFileNameW(nullptr, pathChar, MAX_PATH);

    char ch[260];
    char defChar = ' ';
    WideCharToMultiByte(CP_ACP,0,pathChar,-1, ch,260,&defChar, nullptr);

    // A std:string  using the char* constructor.
    std::string path(ch);
    std::string executablePath = path.substr(0, path.find_last_of('\\'));

    return executablePath;
}
