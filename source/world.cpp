/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"

World::World(programArguments& args)
    : m_programArgs(args)
{

    systemTypeStringToEnum = {
            {"FREE", FREE},
            {"V", V},
            {"Y", Y}
    };

}

void World::update() {

    if (m_programArgs.sitlMode) {
        m_system->updateSitl(simulationConfiguration.dt.count());
    } else {
        m_system->update(simulationConfiguration.dt.count());
    }

    logger().loggerClockTick(simulationConfiguration.dt.count());
}

void World::readConfigurationFile(const std::string& fileName) {
    YAML::Node nodeConfiguration = YAML::LoadFile(fileName);
    simulationParameters simParams = ConfigurationParser::parseSimulationParameters(nodeConfiguration);
    m_systemConfiguration = ConfigurationParser::parseSystemConfiguration(nodeConfiguration);

    m_logDirectory = simParams.logFilesDirectory;
    m_logger = std::make_unique<Logger>(m_programArgs.loggingEnabled, m_logDirectory);

    std::chrono::duration<double> timeStep(1.0 / m_systemConfiguration.llcRate);
    simulationConfiguration = {simParams.fixedDuration, simParams.duration, timeStep};

    m_systemProperties.fixedWing = ConfigurationParser::parseFixedWingProperties(simParams.aircraftPropertiesFileDirectory);
    m_ctrlGains = ConfigurationParser::parseControllerGains(simParams.controllerGainsFileDirectory);

    if (m_systemConfiguration.configuration != "FREE") {
        m_systemProperties.tether = ConfigurationParser::parseTetherProperties(simParams.tetherPropertiesFileDirectory);

        m_systemProperties.payload = ConfigurationParser::parsePayloadProperties(simParams.payloadPropertiesFileDirectory);
        m_initialPayloadStates = ConfigurationParser::parseInitialStates(nodeConfiguration, "InitialStatesPayload");
    }

    if (m_systemConfiguration.haveLauncher) {
        m_systemProperties.launcher = ConfigurationParser::parseLauncherProperties(simParams.launcherPropertiesFileDirectory);
    }

    for (int i = 1; i <= m_systemConfiguration.numberOfUav; i++) {
        std::string nodeName = "InitialStatesAircraft_" + std::to_string(i);
        initialStates states = ConfigurationParser::parseInitialStates(nodeConfiguration, nodeName);
        m_initialAircraftStatesVector.push_back(states);
    }

    m_systemTarget = ConfigurationParser::parseSystemTarget(nodeConfiguration);

    std::vector<float> wind = ConfigurationParser::parseWind(fileName);
    physics.updateWind({wind[0], wind[1], wind[2]});
}

Logger& World::logger() {
    return *m_logger;
}

void World::createSystem() {

    m_system = std::make_unique<System>(m_programArgs, physics, logger(), m_systemConfiguration, m_systemProperties, m_ctrlGains);
    m_system->setTarget(m_systemTarget);

    switch(getSystemType()) {
        case FREE:
            switch (m_systemConfiguration.numberOfUav) {
                case 1:
                    m_system->createSystemOneAircraft(m_initialAircraftStatesVector.front());
                    break;
                case 2:
                    m_system->createSystemTwoAircraft(m_initialAircraftStatesVector);
                    break;
                case 3:
                    throw std::invalid_argument( "System 3 aircraft free flying not implemented" );
                default:
                    throw std::invalid_argument( "Invalid number of UAV" );
            }
            break;

        case V:
            switch (m_systemConfiguration.numberOfUav) {
                case 1:
                    m_system->createSystemOneAircraftGrounded(m_initialAircraftStatesVector.front());
                    break;
                case 2:
                    m_system->createSystemTwoAircraftV(m_systemConfiguration.grounded, m_initialAircraftStatesVector, m_initialPayloadStates);
                    break;
                case 3:
                    m_system->createSystemThreeAircraftV(m_systemConfiguration.grounded, m_initialAircraftStatesVector, m_initialPayloadStates);
                    break;
                default:
                    throw std::invalid_argument( "Invalid number of UAV" );
            }
            break;

        case Y:
            switch (m_systemConfiguration.numberOfUav) {
                case 1:
                    throw std::invalid_argument( "Invalid number of UAV" );
                case 2:
                    m_system->createSystemTwoAircraftY();
                    break;
                case 3:
                    throw std::invalid_argument( "System 3 aircraft Y tethers not implemented" );
                default:
                    throw std::invalid_argument( "Invalid number of UAV" );
            }
    }
}

void World::resetSystem() {
    m_logger.reset();
    m_system.reset();

    m_logger = std::make_unique<Logger>(m_programArgs.loggingEnabled, m_logDirectory);
    createSystem();
}

void World::deleteSystem() {
    m_logger.reset();
    m_system.reset();

    m_initialAircraftStatesVector.clear();
}

systemType World::getSystemType() const {
    return systemTypeStringToEnum.at(m_systemConfiguration.configuration);
}

int World::getSystemNumberOfUav() const {
    return m_systemConfiguration.numberOfUav;
}

System* World::system() {
    return m_system.get();
}

#if defined(_WIN32) || defined(_WIN64) // Windows-specific implementation

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

#elif defined(__linux__) // Linux-specific implementation

std::string World::getExecutablePath() {
    char path[PATH_MAX];
    const ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    if (count == -1) {
        throw std::runtime_error("Failed to get executable path");
    }

    // Convert to std::string
    const std::string fullPath(path, count);
    // Extract the directory path
    std::string executablePath = fullPath.substr(0, fullPath.find_last_of('/'));

    return executablePath;
}

#else
#error "Unsupported platform"
#endif

