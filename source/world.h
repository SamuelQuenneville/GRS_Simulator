/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WORLD_H
#define GRS_SIMULATOR_WORLD_H

#include <map>

#include "configurationParser.h"
#include "system.h"

class World {

public:
    World();

    [[nodiscard]] bool isRunning() const {return m_isRunning;};
    void start() {m_isRunning = true;};
    void stop() {m_isRunning = false;};

    worldPhysics worldPhysics;

    struct simulationConfiguration {
        bool fixedDuration;
        float duration;
        double dt;
    };

    simulationConfiguration simulationConfiguration{};

    void update();
    void readConfigurationFile(const std::string& fileName);
    Logger& logger();

    void createSystem();

    std::map<std::string, systemType> systemTypeStringToEnum;
    static int getNumberOfAircraftInSystem(systemType type);

private:
    std::unique_ptr<Logger> m_logger;

    bool m_isRunning = true;
    std::string m_logDirectory;

    systemConfiguration m_systemConfiguration{};
    fixedWingProperties m_aircraftProperties{};
    tetherProperties m_tetherProperties{};

    std::vector<initialStates> m_initialAircraftStatesVector;
    initialStates m_initialPayloadStates;
    target m_systemTarget;

    std::unique_ptr<System> m_system;
};


#endif //GRS_SIMULATOR_WORLD_H
