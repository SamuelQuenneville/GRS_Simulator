/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "oneAircraftGrounded.h"

OneAircraftGrounded::OneAircraftGrounded(World *world)
    : Scene()
    , m_world(world)
    , m_ground(nullptr)
    , mVBOVertices(GL_ARRAY_BUFFER)
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
        auto* aircraftObject = new Aircraft(aircraft.get(), {2, 2, 2});
        aircraftObject->setColor(Color::red());
        m_physicsObjects.push_back(aircraftObject);
    }

    // ------------ TETHER ----------- //
    m_tether = new TetherLine();

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
    for (auto & physicsObject : m_physicsObjects) {
        physicsObject->setTransform(Transform({0, 1.382, 30}, Quaternion::fromEulerAngles(0, 0,radians(18.5))));
        m_tether->setPosition({0,0,0}, cast<float>(physicsObject->getTransform().getPosition()));
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

void OneAircraftGrounded::renderSinglePass(Shader &shader, const Matrix4<double> &worldToCameraMatrix) {

    //----------
    mLinePoints.clear();
    m_tether->setPosition({0.0f,0.0f,0.0f}, cast<float>(m_physicsObjects.at(0)->getTransform().getPosition()));
    mLinePoints.emplace_back(m_tether->getPoint1());
    mLinePoints.emplace_back(m_tether->getPoint2());

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    auto sizeVertices = GLsizei(mLinePoints.size() * sizeof(Vector3<float>));
    mVBOVertices.copyDataIntoVBO(sizeVertices, &mLinePoints[0], GL_STATIC_DRAW);
    mVBOVertices.unbind();

    mVAO.create();
    mVAO.bind();
    mVBOVertices.bind();
    mVAO.unbind();


    //----------
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    mVAO.bind();
    m_colorShader.bind();
    mVBOVertices.bind();

    // Set the model to camera matrix
    m_colorShader.setMatrix4x4Uniform("localToWorldMatrix", Matrix4<double>::identity());
    m_colorShader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    Vector4<double> color(0, 0.0, 0, 1);
    m_colorShader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    m_colorShader.setVector4Uniform("globalVertexColor", color, false);

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = m_colorShader.getAttribLocation("vertexPosition");

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)nullptr);

    // Draw the lines
    glDrawArrays(GL_LINES, 0, mLinePoints.size() * 2);
    glDisableVertexAttribArray(vertexPositionLoc);

    mVBOVertices.unbind();
    mVAO.unbind();
    m_colorShader.unbind();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (auto & m_physicsObject : m_physicsObjects) {
        m_physicsObject->render(shader, worldToCameraMatrix);
    }

    // Render the static objects (floor could be obstacle...)
    for (auto & m_staticObject : m_staticObjects) {
        m_staticObject->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}

