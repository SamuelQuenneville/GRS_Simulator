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
    , m_floor(nullptr)
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

    m_aircraft1 = new Aircraft({0.005, 0.005, 0.005});
    m_aircraft1->setColor(Color(1.0f, 0.0f, 0.0f, 1.0f));
    m_physicsObjects.push_back(m_aircraft1);

    // ---------- FLOOR ---------- //
    m_floor = new Cube({60,0.5,60});
    m_floor->setColor(Color::floorColor());
    m_physicsObjects.push_back(m_floor);
}

void OneAircraftGrounded::destroyPhysicsWorld() {
    if (m_world != nullptr) {
        // Destroy all the physics objects of the scene
        for (auto & m_physicsObject : m_physicsObjects) {
            delete m_physicsObject;
        }

        for (auto & m_staticObject : m_staticObjects) {
            delete m_staticObject;
        }

        m_physicsObjects.clear();
        m_staticObjects.clear();
        m_world = nullptr;
    }
}

void OneAircraftGrounded::initBodiesPositions() {
    m_aircraft1->setTransform(Transform({15, 10, 0}, Quaternion{0.5,0.5,0.5,-0.5}));
    m_floor->setTransform(Transform({0.0,0.0,0.0}, Quaternion::identity()));
}

void OneAircraftGrounded::updatePhysics() {
    // Take a simulation step
    m_world->update();
}

void OneAircraftGrounded::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

