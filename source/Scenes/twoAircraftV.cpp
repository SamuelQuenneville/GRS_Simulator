/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "twoAircraftV.h"

TwoAircraftV::TwoAircraftV(World& world)
    : Scene(World::getExecutablePath())
    , m_world(world)
{
    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();
}

TwoAircraftV::~TwoAircraftV() {

}

void TwoAircraftV::createPhysicsWorld() {

    // ---------- AIRCRAFT ---------- //
    for (const auto& aircraft : m_world.system()->aircraft()) {
        auto aircraftObject = std::make_unique<Aircraft>(aircraft.get(), Vector3<double>(2.0, 2.0, 2.0), World::getExecutablePath());
        aircraftObject->setColor(Color::color(aircraft->getId()));

        m_physicsObjects.push_back(std::move(aircraftObject));
    }

    // -----------  PAYLOAD ---------- //
    auto payloadObject = std::make_unique<PayloadObject>(m_world.system()->payload().get(), Vector3<float>(0.4,0.4,0.4), World::getExecutablePath());
    payloadObject->setColor(Color::blue());
    m_physicsObjects.push_back(std::move(payloadObject));

    // ------------ TETHER ----------- //
    for (const auto& tether : m_world.system()->tethers()) {
        auto tetherObject = std::make_unique<TetherLine>(tether.get());
        m_tethersObjects.push_back(std::move(tetherObject));
    }

    // ---------- LAUNCHER ---------- //
    m_launcher1 = std::make_shared<Launcher>(Vector3<double>(1.0,1.0,1.0), World::getExecutablePath());
    m_launcher1->setColor(Color::backgroundColor());
    m_launcher2 = std::make_shared<Launcher>(Vector3<double>(1.0,1.0,1.0), World::getExecutablePath());
    m_launcher2->setColor(Color::backgroundColor());
    m_staticObjects.push_back(m_launcher1);
    m_staticObjects.push_back(m_launcher2);

    // ---------- GROUND ---------- //
    m_ground = std::make_shared<Cube>(Vector3<float>(50.0, 0.5, 50.0), World::getExecutablePath());
    m_ground->setColor(Color::floorColor());
    m_staticObjects.push_back(m_ground);

    m_target = std::make_shared<Cube>(Vector3<float>(0.4, 0.4, 0.4), World::getExecutablePath());
    m_target->setColor({1.0f, 0.0f, 0.0f, 0.6f});
    m_staticObjects.push_back(m_target);
}

void TwoAircraftV::destroyPhysicsWorld() {
    // Destroy all the physics objects of the scene
    m_physicsObjects.clear();
    m_tethersObjects.clear();
    m_staticObjects.clear();
}

void TwoAircraftV::initBodiesPositions() {
    m_launcher1->setTransform(Transform({-0.849, 0.816, 30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0), 0.0, M_PI/2.0)));
    m_launcher2->setTransform(Transform({0.849, 0.816, -30.0}, Quaternion::fromEulerAngles(M_PI + radians(19.0),0.0, -M_PI/2.0)));
    m_ground->setTransform(Transform({0.0, -0.5, 0.0}, Quaternion::identity()));

    m_target->setTransform(Transform({0.0,0.0,0.0}, Quaternion::identity()));
}

void TwoAircraftV::updatePhysics() {
    m_world.update();
}

void TwoAircraftV::updateTarget(double groundspeed, std::vector<double> payloadPosition) {
    target t = m_world.system()->getTarget();
    t.groundspeed = static_cast<float>(groundspeed);
    t.positionPayload = payloadPosition;
    m_world.system()->setTarget(t);

    m_target->setTransform(Transform({payloadPosition[0], payloadPosition[2],payloadPosition[1]}, Quaternion::identity()));
}

void TwoAircraftV::reset() {
    Scene::reset();
    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

void TwoAircraftV::destroy() {
    Scene::destroy();
    destroyPhysicsWorld();
}

void TwoAircraftV::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {
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
