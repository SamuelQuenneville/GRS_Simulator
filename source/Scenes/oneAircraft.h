/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_ONEAIRCRAFT_H
#define GRS_SIMULATOR_ONEAIRCRAFT_H

#include "scene.h"
#include "world.h"

class OneAircraft : public Scene {
public:
    explicit OneAircraft(World* world);
    ~OneAircraft() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions();

    void updatePhysics() override;
    void updateTarget(double groundspeed, std::vector<double> payloadPosition) override;
    void reset() override;
    void destroy() override;

    void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

protected:
    World* m_world;
    Cube* m_ground{};

};


#endif //GRS_SIMULATOR_ONEAIRCRAFT_H
