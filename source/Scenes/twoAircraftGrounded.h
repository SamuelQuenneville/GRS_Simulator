/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TWOAIRCRAFTGROUNDED_H
#define GRS_SIMULATOR_TWOAIRCRAFTGROUNDED_H

#include "scene.h"
#include "world.h"

class TwoAircraftGrounded : public Scene {

public:
    explicit TwoAircraftGrounded(World* world);
    ~TwoAircraftGrounded() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions();

    void updatePhysics() override;
    void reset() override;

    void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

protected:
    World* m_world;

    Cube* m_ground{};
    Launcher* m_launcher1{};
    Launcher* m_launcher2{};
};


#endif //GRS_SIMULATOR_TWOAIRCRAFTGROUNDED_H
