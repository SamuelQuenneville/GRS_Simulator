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
#include "RigidBodyAndLink/SceneObjects/launcher.h"
#include "RigidBodyAndLink/SceneObjects/tetherLine.h"

class OneAircraftGrounded : public Scene {

public:
    explicit OneAircraftGrounded(World* world);
    ~OneAircraftGrounded() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions();

    void updatePhysics() override;
    void reset() override;

    void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

protected:
    World* m_world;

    Cube* m_ground{};
    Launcher* m_launcher{};
};


#endif //GRS_SIMULATOR_ONEAIRCRAFTGROUNDED_H
