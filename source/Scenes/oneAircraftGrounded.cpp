/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneAircraftGrounded.h"

OneAircraftGrounded::OneAircraftGrounded(World& world)
    : Scene(World::getExecutablePath())
    , m_world(world)
{
    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();
}

OneAircraftGrounded::~OneAircraftGrounded() {

}

void OneAircraftGrounded::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world.system()->aircraft()) {
        auto aircraftObject = std::make_unique<Aircraft>(aircraft.get(), Vector3<double>(2, 2, 2), World::getExecutablePath());
        aircraftObject->setColor(Color::red());
        m_physicsObjects.push_back(std::move(aircraftObject));
    }

    // ------------ TETHER ----------- //
    for (const auto& tether : m_world.system()->tethers()) {
        auto tetherObject = std::make_unique<TetherLine>(tether.get());
        m_tethersObjects.push_back(std::move(tetherObject));
    }

    // ---------- LAUNCHER ---------- //
    m_launcher = std::make_shared<Launcher>(Vector3<double>(1,1,1), World::getExecutablePath());
    m_launcher->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher);

    // ---------- GROUND ---------- //
    m_ground = std::make_shared<Cube>(Vector3<float>(50, 0.5, 50), World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void OneAircraftGrounded::destroyPhysicsWorld() {
    // Destroy all the physics objects of the scene
    m_physicsObjects.clear();
    m_tethersObjects.clear();
    m_staticObjects.clear();
}

void OneAircraftGrounded::initBodiesPositions() {
    m_launcher->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));
}

void OneAircraftGrounded::updatePhysics() {
    m_world.update();
}

void OneAircraftGrounded::updateTarget(double groundspeed, std::vector<double> payloadPosition) {
    target t = m_world.system()->getTarget();
    t.groundspeed = static_cast<float>(groundspeed);
    t.positionPayload = payloadPosition;
    m_world.system()->setTarget(t);
}

void OneAircraftGrounded::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void OneAircraftGrounded::destroy() {
    Scene::destroy();
    destroyPhysicsWorld();
}

void OneAircraftGrounded::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {

    for (auto& tethersObject : m_tethersObjects) {
        tethersObject->render(m_colorShader, worldToCameraMatrix);
    }

    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (auto& physicsObject : m_physicsObjects) {
        physicsObject->render(shader, worldToCameraMatrix);
    }

    // Render the static objects (floor could be obstacle...)
    for (auto& staticObject : m_staticObjects) {
        staticObject->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}
