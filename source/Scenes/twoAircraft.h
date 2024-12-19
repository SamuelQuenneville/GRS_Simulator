/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TWOAIRCRAFT_H
#define GRS_SIMULATOR_TWOAIRCRAFT_H

#include "scene.h"
#include "world.h"

class TwoAircraft : public Scene {
public:
    explicit TwoAircraft(World* world);
    ~TwoAircraft() override;

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
    Launcher* m_launcher1{};
    Launcher* m_launcher2{};
};


#endif //GRS_SIMULATOR_TWOAIRCRAFT_H
