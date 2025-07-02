/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_THREEAIRCRAFTV_H
#define GRS_SIMULATOR_THREEAIRCRAFTV_H

#include "scene.h"
#include "world.h"

class ThreeAircraftV final : public Scene {

public:
    explicit ThreeAircraftV(World& world);
    ~ThreeAircraftV() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions() const;

    void updatePhysics() override;
    void updateTarget(float groundspeed,  std::vector<double> payloadPosition) override;
    void reset() override;
    void destroy() override;

    void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

    void setPauseTimerCallback(std::function<void()> callback) override;

protected:
    World& m_world;

    std::shared_ptr<Cube> m_ground;
    std::shared_ptr<Cube> m_target;
    std::shared_ptr<Launcher> m_launcher1;
    std::shared_ptr<Launcher> m_launcher2;
    std::shared_ptr<Launcher> m_launcher3;
};


#endif //GRS_SIMULATOR_THREEAIRCRAFTV_H
