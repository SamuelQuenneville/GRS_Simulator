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

#include "Configuration/configurationParser.h"
#include "system.h"

class World {

public:
    explicit World(programArguments& args);

    [[nodiscard]] bool isRunning() const {return m_isRunning;};
    void start() {m_isRunning = true;};
    void stop() {m_isRunning = false;};

    worldPhysics physics;

    struct simulationConfiguration {
        bool fixedDuration;
        float duration;
        std::chrono::duration<double> dt;
    };

    simulationConfiguration simulationConfiguration{};

    void update();
    void readConfigurationFile(const std::string& fileName);
    Logger& logger();

    void createSystem();
    void resetSystem();
    void deleteSystem();

    std::map<std::string, systemType> systemTypeStringToEnum;
    [[nodiscard]] systemType getSystemType() const;
    [[nodiscard]] int getSystemNumberOfUav() const;
    System* system();

    static std::string getExecutablePath();

private:
    programArguments& m_programArgs;

    std::unique_ptr<Logger> m_logger;

    bool m_isRunning = true;
    std::string m_logDirectory;

    systemConfiguration m_systemConfiguration{};
    fixedWingProperties m_aircraftProperties{};
    tetherProperties m_tetherProperties{};
    payloadProperties m_payloadProperties{};
    ctrlGains m_ctrlGains{};

    std::vector<initialStates> m_initialAircraftStatesVector;
    initialStates m_initialPayloadStates;
    target m_systemTarget{};

    std::unique_ptr<System> m_system;
};

#endif //GRS_SIMULATOR_WORLD_H