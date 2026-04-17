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
    , m_elapsedTime(0.0)
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

    m_elapsedTime += simulationConfiguration.dt.count();
}

std::string World::getDateString() {
    time_t rawTime;
    char buffer[80];

    time(&rawTime);
    const struct tm *timeInfo = localtime(&rawTime);

    strftime(buffer,sizeof(buffer),"/%Y-%m-%d_%H-%M-%S",timeInfo);
    const std::string date(buffer);
    return date;
}

void World::readConfigurationFile(const std::string& fileName) {
    YAML::Node nodeConfiguration = YAML::LoadFile(fileName);
    const simulationParameters simParams = ConfigurationParser::parseSimulationParameters(nodeConfiguration);
    m_systemConfiguration = ConfigurationParser::parseSystemConfiguration(nodeConfiguration);

    m_logDirectory = simParams.logFilesDirectory;
    Logger::instance().start(m_programArgs.loggingEnabled, m_logDirectory + getDateString());

    const std::chrono::duration<double> timeStep(1.0 / m_systemConfiguration.llcRate);
    simulationConfiguration = {timeStep};

    m_systemProperties.fixedWing = ConfigurationParser::parseFixedWingProperties(simParams.aircraftPropertiesFileDirectory);
    m_ctrlGains = ConfigurationParser::parseControllerGains(simParams.controllerGainsFileDirectory);

    if (m_systemConfiguration.configuration != "FREE") {
        m_systemProperties.tether = ConfigurationParser::parseTetherProperties(simParams.tetherPropertiesFileDirectory);

        m_systemProperties.payload = ConfigurationParser::parsePayloadProperties(simParams.payloadPropertiesFileDirectory);
        m_systemProperties.tetherNode = ConfigurationParser::parseNodeProperties(simParams.payloadPropertiesFileDirectory);
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
    if (getSystemType() == Y) {
        m_systemTarget.altOffsetY = m_systemProperties.tether.byName("Vertical").length;
    }

    std::vector<float> wind = ConfigurationParser::parseWind(fileName);
    physics.updateWind({wind[0], wind[1], wind[2]});
}

void World::createSystem() {

    m_system = std::make_unique<System>(m_programArgs, physics, m_systemConfiguration, m_systemProperties, m_ctrlGains);
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
                    m_system->createSystemTwoAircraftY(m_systemConfiguration.grounded, m_initialAircraftStatesVector, m_initialPayloadStates);
                    break;
                case 3:
                    throw std::invalid_argument( "System 3 aircraft Y tethers not implemented" );
                default:
                    throw std::invalid_argument( "Invalid number of UAV" );
            }
    }

    // Initialize a snapshot for scene creation
    publishSnapshot(system()->getRenderData());
}

void World::resetSystem() {
    Logger::instance().stop();
    m_system.reset();

    Logger::instance().start(m_programArgs.loggingEnabled, m_logDirectory + getDateString());
    createSystem();
}

void World::deleteSystem() {
    Logger::instance().stop();
    m_system.reset();

    m_initialAircraftStatesVector.clear();
}

std::vector<Frame*> World::collectFrames() const {
    std::vector<Frame*> frames;

    for (auto&& aircraft : m_system->aircraft()) {
        auto f = aircraft->frames();
        frames.insert(frames.end(), f.begin(), f.end());
    }

    if (m_system->tetherNode()) {
        auto f = m_system->tetherNode()->frames();
        frames.insert(frames.end(), f.begin(), f.end());
    }

    if (m_system->payload()) {
        auto f = m_system->payload()->frames();
        frames.insert(frames.end(), f.begin(), f.end());
    }

    return frames;
}

void World::setPauseTimerCallback(std::function<void()> callback) const {
    m_system->setPauseTimerCallback(std::move(callback));
}

systemType World::getSystemType() const {
    return systemTypeStringToEnum.at(m_systemConfiguration.configuration);
}

std::shared_ptr<System> World::system() const {
    return m_system;
}

sceneConfiguration World::getSceneConfiguration() const {
    sceneConfiguration config;
    config.configuration = m_systemConfiguration.configuration;
    config.haveLauncher  = m_systemConfiguration.haveLauncher;
    config.haveTarget    = m_systemConfiguration.haveTarget;

    return config;
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

void World::startPhysicsThread() {
    if (m_running.load()) return;

    m_running.store(true);
    m_physicsThread = std::thread(&World::m_physicsLoop, this);
}

void World::stopPhysicsThread() {
    m_running.store(false);

    if (m_physicsThread.joinable()) {
        m_physicsThread.join();
    }
}

std::shared_ptr<systemRenderData> World::getLatestSnapshot() const {
    return std::atomic_load_explicit(&m_snapshot, std::memory_order_acquire);
}

void World::publishSnapshot(const std::shared_ptr<systemRenderData>& snapshot) {
    snapshot->elapsedTime = m_elapsedTime;
    std::atomic_store_explicit(&m_snapshot, snapshot, std::memory_order_release);
}

void World::m_physicsLoop() {

    const auto dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(simulationConfiguration.dt);
    auto next = std::chrono::steady_clock::now();

    while (m_running.load()) {
        next += dt;

        // ---- PHYSICS STEP ----
        if (isRunning()) {
            update();
        }

        // ---- SNAPSHOT ----
        if (const auto sys = system()) {
            publishSnapshot(sys->getRenderData());
        }

        auto now = std::chrono::steady_clock::now();
        if (now < next) {
            std::this_thread::sleep_until(next);
        } else {
            // overrun → resync
            next = now;
        }
    }
}

#else
#error "Unsupported platform"
#endif

