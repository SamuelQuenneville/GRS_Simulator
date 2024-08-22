/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TWOAIRCRAFTV_H
#define GRS_SIMULATOR_TWOAIRCRAFTV_H

#include "scene.h"
#include "world.h"

class TwoAircraftV : public Scene {

public:
    explicit TwoAircraftV(World* world);
    ~TwoAircraftV() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions();

    void updatePhysics() override;
    void updateTarget(double groundspeed,  std::vector<double> payloadPosition) override;
    void reset() override;

    void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

protected:
    World* m_world;

    PayloadObject* m_payload;

    Cube* m_ground{};
    Cube* m_target{};
    Launcher* m_launcher1{};
    Launcher* m_launcher2{};
};


#endif //GRS_SIMULATOR_TWOAIRCRAFTV_H
