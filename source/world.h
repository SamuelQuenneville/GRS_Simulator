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
#include "Mathematics/vector3.h"
#include "Definitions/worldPhysics.h"
#include "system.h"

class World {

public:
    World();

    [[nodiscard]] bool isRunning() const {return m_isRunning;};
    void start() {m_isRunning = true;};

    worldPhysics worldPhysics;

    enum systemType {
        ONE_AIRCRAFT,
        ONE_AIRCRAFT_GROUNDED,
        TWO_AIRCRAFT_GROUNDED_V,
        TWO_AIRCRAFT_FREE_V,
        TWO_AIRCRAFT_GROUNDED_Y,
        TWO_AIRCRAFT_FREE_Y,
    };

    struct simulationConfiguration {
        bool fixedDuration;
        float duration;
    };

    struct systemConfiguration {
        systemType type;
        int hlcRate;
        int llcRate;
        bool windOn;
        bool tetherAerodynamicOn;
    };

    void update();
    void readConfigurationFile(const std::string& fileName);
    void log();

    void createSystem();

    std::map<std::string,systemType> systemTypeStringToEnum;
    static int getNumberOfAircraftInSystem(systemType type);

private:
    bool m_isRunning = false;
    simulationConfiguration m_simulationConfiguration{};
    systemConfiguration m_systemConfiguration{};
    fixedWingProperties m_aircraftProperties{};
    tetherProperties m_tetherProperties{};

    std::vector<initialStates*> m_initialAircraftStatesVector;
    initialStates m_initialPayloadStates;
    target m_systemTarget;

    std::unique_ptr<System> m_system;

};


#endif //GRS_SIMULATOR_WORLD_H
