/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WORLD_H
#define GRS_SIMULATOR_WORLD_H

#if defined(_WIN32) || defined(_WIN64) // Windows-specific implementation
#include <windows.h>
#elif defined(__linux__) // Linux-specific implementation
#include <unistd.h>
#include <climits>
#endif

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "Definitions/renderObjectDefinitions.h"
#include "Configuration/configurationParser.h"
#include "WindModel/windEnvironment.h"
#include "system.h"

class World {

public:
    explicit World(programArguments& args);

    [[nodiscard]] bool isRunning() const {return m_isRunning.load();};
    void start() {m_isRunning.store(true);};
    void stop() {m_isRunning.store(false);};

    worldPhysics physics;

    struct simulationConfiguration {
        std::chrono::duration<double> dt;
    };

    simulationConfiguration simulationConfiguration{};

    void update();
    [[nodiscard]] static std::string getDateString() ;
    void readConfigurationFile(const std::string& fileName);

    void createSystem();
    void resetSystem();
    void deleteSystem();

    // Commands from the GUI thread, applied by the physics thread before its next step
    void requestTarget(const target& newTarget);
    void requestLaunch();
    [[nodiscard]] target initialTarget() const {return m_systemTarget;};

    // True once after the system reported the end of the run (SITL endSimulation flags)
    [[nodiscard]] bool consumeEndOfRun() {return m_endOfRun.exchange(false);};
    [[nodiscard]] double elapsedTime() const {return m_elapsedTime.load();};

    std::map<std::string, systemType> systemTypeStringToEnum;
    [[nodiscard]] systemType getSystemType() const;
    [[nodiscard]] std::shared_ptr<System> system() const;

    sceneConfiguration getSceneConfiguration() const;

    static std::string getExecutablePath();

    void startPhysicsThread();
    void stopPhysicsThread();
    [[nodiscard]] std::shared_ptr<systemRenderData> getLatestSnapshot() const;
    void publishSnapshot(const std::shared_ptr<systemRenderData>& snapshot);

private:
    programArguments& m_programArgs;

    std::atomic<bool> m_isRunning{false};
    std::string m_logDirectory;

    void m_physicsLoop();
    void m_applyPendingCommands();
    std::thread m_physicsThread;
    std::atomic<bool> m_running{false};
    std::shared_ptr<systemRenderData> m_snapshot;
    std::atomic<double> m_elapsedTime{0.0};
    std::atomic<bool> m_endOfRun{false};

    std::mutex m_commandMutex;
    std::optional<target> m_pendingTarget;
    std::atomic<bool> m_pendingLaunch{false};

    std::unique_ptr<WindEnvironment> m_windEnv;

    systemConfiguration m_systemConfiguration{};
    systemProperties m_systemProperties{};
    ctrlGains m_ctrlGains{};

    std::vector<initialStates> m_initialAircraftStatesVector;
    initialStates m_initialPayloadStates;
    target m_systemTarget{};

    std::shared_ptr<System> m_system;
};

#endif //GRS_SIMULATOR_WORLD_H