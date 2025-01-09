/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "threeAircraftV.h"

ThreeAircraftV::ThreeAircraftV(World *world)
    : Scene(World::getExecutablePath())
    , m_world(world)
    , m_ground(nullptr)
    , m_payload(nullptr)
{
    // Set the center of the scene
    m_setScenePosition({0.0, 40.0, 20.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();
}

ThreeAircraftV::~ThreeAircraftV() {
    destroyPhysicsWorld();
}

void ThreeAircraftV::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world->system()->aircraft()) {
        auto* aircraftObject = new Aircraft(aircraft.get(), {2.0, 2.0, 2.0}, World::getExecutablePath());
        aircraftObject->setColor(Color::color(aircraft->getId()));

        m_physicsObjects.push_back(aircraftObject);
    }

    // -----------  PAYLOAD ---------- //
    m_payload = new PayloadObject(m_world->system()->payload().get(), {0.4,0.4,0.4}, World::getExecutablePath());
    m_payload->setColor(Color::blue());
    m_physicsObjects.push_back(m_payload);

    // ------------ TETHER ----------- //
    for (const auto& tether : m_world->system()->tethers()) {
        auto* tetherObject = new TetherLine(tether.get());
        m_tethersObjects.push_back(tetherObject);
    }

    // ---------- LAUNCHER ---------- //
    m_launcher1 = new Launcher({1.0,1.0,1.0}, World::getExecutablePath());
    m_launcher1->setColor(Color::backgroundColor());
    m_launcher2 = new Launcher({1.0,1.0,1.0}, World::getExecutablePath());
    m_launcher2->setColor(Color::backgroundColor());
    m_launcher3 = new Launcher({1.0,1.0,1.0}, World::getExecutablePath());
    m_launcher3->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher1);
    m_staticObjects.push_back(m_launcher2);
    m_staticObjects.push_back(m_launcher3);

    // ---------- GROUND ---------- //
    m_ground = new Cube({50.0, 0.5, 50.0}, World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);

    m_target = new Cube({0.4, 0.4, 0.4}, World::getExecutablePath());
    m_target->setColor({1.0f, 0.0f, 0.0f, 0.6f});
    m_staticObjects.push_back(m_target);
}

void ThreeAircraftV::destroyPhysicsWorld() {
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

void ThreeAircraftV::initBodiesPositions() {
    m_launcher1->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_launcher2->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_launcher3->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));

    m_launcher2->rotateAroundWorldPoint({0,1,0},M_2PI/3,{0,0,0});
    m_launcher3->rotateAroundWorldPoint({0,1,0},-M_2PI/3,{0,0,0});

    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));

    m_payload->setTransform(Transform({0.0,0.0,0.0}, Quaternion::identity()));
    m_target->setTransform(Transform({0.0,0.0,0.0}, Quaternion::identity()));
}

void ThreeAircraftV::updatePhysics() {
    m_world->update();
}

void ThreeAircraftV::updateTarget(double groundspeed, std::vector<double> payloadPosition) {
    target t = m_world->system()->getTarget();
    t.groundspeed = static_cast<float>(groundspeed);
    t.positionPayload = payloadPosition;
    m_world->system()->setTarget(t);

    m_target->setTransform(Transform({payloadPosition[0], payloadPosition[2],payloadPosition[1]}, Quaternion::identity()));
}

void ThreeAircraftV::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void ThreeAircraftV::destroy() {
    Scene::destroy();
    destroyPhysicsWorld();
}

void ThreeAircraftV::renderSinglePass(Shader &shader, const Matrix4<double> &worldToCameraMatrix) {
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
