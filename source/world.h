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

#include <functional>

#include "Definitions/renderObjectDefinitions.h"
#include "Configuration/configurationParser.h"
#include "WindModel/windEnvironment.h"
#include "system.h"

class World {

public:
    explicit World(programArguments& args);

    [[nodiscard]] bool isRunning() const {return m_isRunning;};
    void start() {m_isRunning = true;};
    void stop() {m_isRunning = false;};

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

    [[nodiscard]] std::vector<Frame*> collectFrames() const;

    void setPauseTimerCallback(std::function<void()> callback) const;

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

    bool m_isRunning = false;
    std::string m_logDirectory;

    void m_physicsLoop();
    std::thread m_physicsThread;
    std::atomic<bool> m_running{false};
    std::shared_ptr<systemRenderData> m_snapshot;
    double m_elapsedTime;

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