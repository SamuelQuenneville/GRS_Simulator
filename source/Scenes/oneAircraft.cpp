/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneAircraft.h"

OneAircraft::OneAircraft(World& world)
        : Scene(World::getExecutablePath())
        , m_world(world)
        , m_ground(nullptr)
{
    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();
}

OneAircraft::~OneAircraft() = default;

void OneAircraft::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world.system()->aircraft()) {
        auto aircraftObject = std::make_unique<Aircraft>(aircraft.get(), Vector3<double>(2, 2, 2), World::getExecutablePath());
        aircraftObject->setColor(Color::color(aircraft->getId()));

        m_physicsObjects.push_back(std::move(aircraftObject));
    }

    // ---------- GROUND ---------- //
    m_ground = std::make_shared<Cube>(Vector3<float>(50, 0.5, 50), World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void OneAircraft::destroyPhysicsWorld() {
    // Destroy all the physics objects of the scene
    m_physicsObjects.clear();
    m_staticObjects.clear();
}

void OneAircraft::initBodiesPositions() const {
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));
}

void OneAircraft::updatePhysics() {
    m_world.update();
}

void OneAircraft::updateTarget(const float groundspeed, const std::vector<double> payloadPosition) {
    target t = m_world.system()->getTarget();
    t.groundspeed = groundspeed;
    t.positionPayload = payloadPosition;
    m_world.system()->setTarget(t);
}

void OneAircraft::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void OneAircraft::destroy() {
    Scene::destroy();
    destroyPhysicsWorld();
}

void OneAircraft::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {
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