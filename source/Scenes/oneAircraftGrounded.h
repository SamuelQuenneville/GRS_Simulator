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

class OneAircraftGrounded final : public Scene {

public:
    explicit OneAircraftGrounded(World& world);
    ~OneAircraftGrounded() override;

    void createPhysicsWorld();
    void destroyPhysicsWorld();

    void initBodiesPositions() const;

    void updatePhysics() override;
    void updateTarget(float groundspeed, std::vector<double> payloadPosition) override;
    void reset() override;
    void destroy() override;

    void renderSinglePass(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) override;

    void setPauseTimerCallback(std::function<void()> callback) override;

protected:
    World& m_world;

    std::shared_ptr<Cube> m_ground;
    std::shared_ptr<Launcher> m_launcher;
};


#endif //GRS_SIMULATOR_ONEAIRCRAFTGROUNDED_H
