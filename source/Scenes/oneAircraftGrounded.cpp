/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneAircraftGrounded.h"

OneAircraftGrounded::OneAircraftGrounded(World* world)
    : Scene(World::getExecutablePath())
    , m_world(world)
    , m_ground(nullptr)
{
    // Set the center of the scene
    m_setScenePosition({0, 40, 20}, 100);
    setInitialZoom(1.5);
    m_resetCameraToViewAll();
}

OneAircraftGrounded::~OneAircraftGrounded() {
    destroyPhysicsWorld();
}

void OneAircraftGrounded::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for(const auto& aircraft : m_world->system()->aircraft()) {
        auto* aircraftObject = new Aircraft(aircraft.get(), {2, 2, 2}, World::getExecutablePath());
        aircraftObject->setColor(Color::red());
        m_physicsObjects.push_back(aircraftObject);
    }

    // ------------ TETHER ----------- //
    for(const auto& tether : m_world->system()->tethers()) {
        auto* tetherObject = new TetherLine(tether.get());
        m_tethersObjects.push_back(tetherObject);
    }

    // ---------- LAUNCHER ---------- //
    m_launcher = new Launcher({1,1,1}, World::getExecutablePath());
    m_launcher->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher);

    // ---------- GROUND ---------- //
    m_ground = new Cube({50, 0.5, 50}, World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void OneAircraftGrounded::destroyPhysicsWorld() {
    if (m_world != nullptr) {
        // Destroy all the physics objects of the scene
        for (auto& physicsObject : m_physicsObjects) {
            delete physicsObject;
        }

        for (auto& staticObject : m_staticObjects) {
            delete staticObject;
        }

        for (auto& tethersObject : m_tethersObjects) {
            delete tethersObject;
        }

        m_physicsObjects.clear();
        m_tethersObjects.clear();
        m_staticObjects.clear();
    }
}

void OneAircraftGrounded::initBodiesPositions() {
    m_launcher->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(-M_PI/2.0,-M_PI/2.0, radians(19.0))));
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));
}

void OneAircraftGrounded::updatePhysics() {
    m_world->update();
}

void OneAircraftGrounded::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void OneAircraftGrounded::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {

    for (auto& tethersObject : m_tethersObjects) {
        tethersObject->render(m_colorShader, worldToCameraMatrix);
    }

    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (auto & physicsObject : m_physicsObjects) {
        physicsObject->render(shader, worldToCameraMatrix);
    }

    // Render the static objects (floor could be obstacle...)
    for (auto & staticObject : m_staticObjects) {
        staticObject->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}
