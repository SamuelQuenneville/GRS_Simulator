/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_WORLD_H
#define GRS_SIMULATOR_WORLD_H

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

    void update();
    void log();

    void createSystem(systemType type);

private:
    bool m_isRunning = false;

    std::unique_ptr<System> m_system;

};


#endif //GRS_SIMULATOR_WORLD_H
