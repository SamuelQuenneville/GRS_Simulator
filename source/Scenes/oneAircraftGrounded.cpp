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
    m_setScenePosition({0, 20, 0}, 100);
    setInitialZoom(1.5);
    m_resetCameraToViewAll();
}

OneAircraftGrounded::~OneAircraftGrounded() {
    destroyPhysicsWorld();
}

void OneAircraftGrounded::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for(const auto& aircraft : m_world->system()->aircraft()) {
        auto* aircraftObject = new Aircraft(aircraft.get(), {0.005, 0.005, 0.005});
        aircraftObject->setColor(Color::red());
        m_physicsObjects.push_back(aircraftObject);
    }

    // ---------- GROUND ---------- //
    m_ground = new Cube({60, 0.5, 60});
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
        physicsObject->setTransform(Transform({0, 0, 30}, Quaternion::fromEulerAngles(-M_PI/2,M_PI,0)));
    }
    m_ground->setTransform(Transform({0.0, -0.85, 0.0}, Quaternion::identity()));
}

void OneAircraftGrounded::updatePhysics() {
    //for (auto & physicsObject : m_physicsObjects) {
    // physicsObject->setTransform(Transform({0, 20, 30}, Quaternion::fromEulerAngles(-M_PI/2,0,0)));
    //}
    m_world->update();
}

void OneAircraftGrounded::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

