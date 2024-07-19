/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_ONEAIRCRAFTGROUNDED_H
#define GRS_SIMULATOR_ONEAIRCRAFTGROUNDED_H

#include "scene.h"
#include "world.h"
#include "RigidBodyAndLink/SceneObjects/cube.h"
#include "RigidBodyAndLink/SceneObjects/aircraft.h"

class OneAircraftGrounded : public Scene {

public:
    explicit OneAircraftGrounded(const std::string& name, World* world);
    ~OneAircraftGrounded() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions();

    void updatePhysics() override;
    void reset() override;

protected:
    // Floor
    Cube* m_ground{};
    Aircraft* m_aircraft;

    World* m_world;


};


#endif //GRS_SIMULATOR_ONEAIRCRAFTGROUNDED_H
