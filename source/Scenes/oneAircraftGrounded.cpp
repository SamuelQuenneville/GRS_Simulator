/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneAircraftGrounded.h"

OneAircraftGrounded::OneAircraftGrounded(const std::string& name, World* world)
    : Scene()
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
        auto* aircraftObject = new Aircraft(aircraft.get(), {1, 1, 1});
        aircraftObject->setColor(Color::red());
        m_physicsObjects.push_back(aircraftObject);
    }

    // ---------- LAUNCHER ---------- //
    m_launcher = new Launcher({1,1,1});
    m_launcher->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher);

    // ---------- GROUND ---------- //
    m_ground = new Cube({50, 0.5, 50});
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);
}

void OneAircraftGrounded::destroyPhysicsWorld() {
    if (m_world != nullptr) {
        // Destroy all the physics objects of the scene
        for (auto & physicsObject : m_physicsObjects) {
            delete physicsObject;
        }

        for (auto & staticObject : m_staticObjects) {
            delete staticObject;
        }

        m_physicsObjects.clear();
        m_staticObjects.clear();
    }
}

void OneAircraftGrounded::initBodiesPositions() {
    // TODO access initial states of each aircraft
    // Quaternion{0.5,0.5,0.5,-0.5}
    for (auto & physicsObject : m_physicsObjects) {
        //physicsObject->setTransform(Transform({0, 0, 30}, Quaternion::identity()));
        physicsObject->setTransform(Transform({0, 1.382, 30}, Quaternion::fromEulerAngles(0, 0,radians(18.5))));
    }
    m_launcher->setTransform(Transform({-0.849, 0.816, 30}, Quaternion::fromEulerAngles(-M_PI/2,-M_PI/2, radians(19))));
    m_ground->setTransform(Transform({0.0, -0.25, 0.0}, Quaternion::identity()));
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

