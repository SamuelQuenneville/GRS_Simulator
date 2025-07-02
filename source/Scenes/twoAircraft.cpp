/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "twoAircraft.h"


TwoAircraft::TwoAircraft(World& world)
        : Scene(World::getExecutablePath())
        , m_world(world)
{
    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();
}

TwoAircraft::~TwoAircraft() = default;

void TwoAircraft::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world.system()->aircraft()) {
        auto aircraftObject = std::make_unique<Aircraft>(aircraft.get(), Vector3<double>(2, 2, 2), World::getExecutablePath());
        aircraftObject->setColor(Color::color(aircraft->getId()));

        m_physicsObjects.push_back(std::move(aircraftObject));
    }

    // ---------- LAUNCHER ---------- //
    m_launcher1 = std::make_shared<Launcher>(Vector3<double>(1.0,1.0,1.0), World::getExecutablePath());
    m_launcher1->setColor(Color::backgroundColor());
    m_launcher2 = std::make_shared<Launcher>(Vector3<double>(1.0,1.0,1.0), World::getExecutablePath());
    m_launcher2->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher1);
    m_staticObjects.push_back(m_launcher2);

    // ---------- GROUND ---------- //
    m_ground = std::make_shared<Cube>(Vector3<float>(50, 0.5, 50), World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void TwoAircraft::destroyPhysicsWorld() {
    // Destroy all the physics objects of the scene
    m_physicsObjects.clear();
    m_staticObjects.clear();
}

void TwoAircraft::initBodiesPositions() const {
    m_launcher1->setTransform(Transform({-0.849, 0.816, 100.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_launcher2->setTransform(Transform({0.849, 0.816, -100.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0),0.0, -M_PI/2.0)));
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));
}

void TwoAircraft::updatePhysics() {
    m_world.update();
}

void TwoAircraft::updateTarget(const float groundspeed, std::vector<double> payloadPosition) {
    target t = m_world.system()->getTarget();
    t.groundspeed = groundspeed;
    t.positionPayload = payloadPosition;
    m_world.system()->setTarget(t);
}

void TwoAircraft::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void TwoAircraft::destroy() {
    Scene::destroy();
    destroyPhysicsWorld();
}

void TwoAircraft::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {
    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (const auto& physicsObject : m_physicsObjects) {
        physicsObject->render(shader, worldToCameraMatrix);
    }

    // Render the static objects (floor could be obstacle...)
    for (const auto& staticObject : m_staticObjects) {
        staticObject->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}

void TwoAircraft::setPauseTimerCallback(std::function<void()> callback) {
    m_world.setPauseTimerCallback(std::move(callback));
}