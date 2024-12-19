/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "twoAircraft.h"


TwoAircraft::TwoAircraft(World *world)
        : Scene(World::getExecutablePath())
        , m_world(world)
        , m_ground(nullptr)
{
    // Set the center of the scene
    m_setScenePosition({0, 40, 20}, 100);
    setInitialZoom(1.5);
    m_resetCameraToViewAll();
}

TwoAircraft::~TwoAircraft() {
    destroyPhysicsWorld();
}

void TwoAircraft::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world->system()->aircraft()) {
        auto* aircraftObject = new Aircraft(aircraft.get(), {2, 2, 2}, World::getExecutablePath());
        if (aircraft->getId() == 1) {
            aircraftObject->setColor(Color::red());
        } else if (aircraft->getId() == 2) {
            aircraftObject->setColor(Color::yellow());
        }

        m_physicsObjects.push_back(aircraftObject);
    }

    // ---------- LAUNCHER ---------- //
    m_launcher1 = new Launcher({1,1,1}, World::getExecutablePath());
    m_launcher1->setColor(Color::backgroundColor());
    m_launcher2 = new Launcher({1,1,1}, World::getExecutablePath());
    m_launcher2->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher1);
    m_staticObjects.push_back(m_launcher2);

    // ---------- GROUND ---------- //
    m_ground = new Cube({50, 0.5, 50}, World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void TwoAircraft::destroyPhysicsWorld() {
    if (m_world != nullptr) {
        // Destroy all the physics objects of the scene
        for (auto& physicsObject : m_physicsObjects) {
            delete physicsObject;
        }

        for (auto& staticObject : m_staticObjects) {
            delete staticObject;
        }

        m_physicsObjects.clear();
        m_staticObjects.clear();
    }
}

void TwoAircraft::initBodiesPositions() {
    m_launcher1->setTransform(Transform({-0.849, 0.816, 100.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_launcher2->setTransform(Transform({0.849, 0.816, -100.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0),0.0, -M_PI/2.0)));
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));
}

void TwoAircraft::updatePhysics() {
    m_world->update();
}

void TwoAircraft::updateTarget(double groundspeed, std::vector<double> payloadPosition) {
    target t = m_world->system()->getTarget();
    t.groundspeed = static_cast<float>(groundspeed);
    t.positionPayload = payloadPosition;
    m_world->system()->setTarget(t);
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
