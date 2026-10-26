/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "world.h"

#include <algorithm>

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

    m_elapsedTime.store(m_elapsedTime.load() + simulationConfiguration.dt.count());
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
    Logger::instance().copyConfigFile(fileName, m_logDirectory + getDateString());

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

    // Wind Environment
    windProperties windParams = ConfigurationParser::parseWind(fileName);

    auto turb = std::make_shared<SpectralWindField>(windParams);
    auto mean = std::make_shared<MeanWind>(windParams);

    m_windEnv = std::make_unique<WindEnvironment>(mean, turb, windParams);
}

void World::createSystem() {

    m_system = std::make_unique<System>(m_programArgs, physics, *m_windEnv, m_systemConfiguration, m_systemProperties, m_ctrlGains);
    m_system->setTarget(m_systemTarget);

    // Called from the physics thread: only set flags, the viewer reacts on its own thread
    m_system->setPauseTimerCallback([this]() {
        m_isRunning.store(false);
        m_endOfRun.store(true);
    });

    // Drop commands aimed at a previous system
    {
        std::lock_guard lock(m_commandMutex);
        m_pendingTarget.reset();
    }
    m_pendingLaunch.store(false);
    m_endOfRun.store(false);
    m_elapsedTime.store(0.0);

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
    // The physics thread must not touch the system while it is replaced
    const bool threadWasRunning = m_running.load();
    stopPhysicsThread();
    stop();

    Logger::instance().stop();
    m_system.reset();

    Logger::instance().start(m_programArgs.loggingEnabled, m_logDirectory + getDateString());
    createSystem();

    if (threadWasRunning) {
        startPhysicsThread();
    }
}

void World::deleteSystem() {
    stopPhysicsThread();
    stop();

    Logger::instance().stop();
    m_system.reset();

    m_initialAircraftStatesVector.clear();
}

void World::requestTarget(const target& newTarget) {
    std::lock_guard lock(m_commandMutex);
    m_pendingTarget = newTarget;
}

void World::requestLaunch() {
    m_pendingLaunch.store(true);
}

void World::m_applyPendingCommands() {
    if (!m_system) return;

    std::optional<target> newTarget;
    {
        std::lock_guard lock(m_commandMutex);
        newTarget.swap(m_pendingTarget);
    }
    if (newTarget) {
        m_system->setTarget(*newTarget);
    }

    if (m_pendingLaunch.exchange(false)) {
        m_system->setLaunchTriggered(true);
    }
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
    snapshot->elapsedTime = m_elapsedTime.load();
    std::atomic_store_explicit(&m_snapshot, snapshot, std::memory_order_release);
}

void World::m_physicsLoop() {

    const auto dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(simulationConfiguration.dt);
    auto next = std::chrono::steady_clock::now();

    // Snapshots are only consumed at the viewer frame rate, so building one
    // every physics step only costs allocations and shared_ptr lock contention.
    const auto publishPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, m_programArgs.renderFps)));
    auto nextPublish = next;

    while (m_running.load()) {
        next += dt;

        // ---- GUI COMMANDS ----
        m_applyPendingCommands();

        // ---- PHYSICS STEP ----
        if (isRunning()) {
            update();
        }

        // ---- SNAPSHOT (rate limited, none in headless mode) ----
        if (const auto tPublish = std::chrono::steady_clock::now(); !m_programArgs.headless && tPublish >= nextPublish) {
            if (const auto sys = system()) {
                publishSnapshot(sys->getRenderData());
            }
            nextPublish = tPublish + publishPeriod;
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

