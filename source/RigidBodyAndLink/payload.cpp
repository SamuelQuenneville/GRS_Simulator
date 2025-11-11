/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "payload.h"

Payload::Payload(const worldPhysics& physics, const payloadProperties& properties, const bool isGrounded)
    : m_physics(physics)
    , m_properties(properties)
    , m_isGrounded(isGrounded)
{
    // Create a cg Frame
    Frame& cg = addFrame("centerOfMass", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::WHITE);
    setCenterOfMassFrame(cg);

    // Create other Frames and assign function
    Frame& aeroReference = addFrame("aeroRef", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::CYAN);
    Frame& tetherAttach  = addFrame("tetherAttach", this->bodyOriginFrame(), {0,0,0}, {0,0,0}, Color::HexColor::YELLOW);

    aeroReference.setLoadCallback([this](const Frame& f) {
        f.setForce(m_calcAeroForces());
        f.setMoment({0.0,0.0,0.0});
    });

    tetherAttach.setLoadCallback([this](const Frame& f) {
        grs::Vec3d tetherForces = grs::Vec3d::zeros();

        const auto connectedNode = connections[&f.originNode()];
        for(const Node* node : connectedNode) {
            tetherForces += node->force();
        }

        f.setForce(tetherForces);
        f.setMoment({0.0,0.0,0.0});
    });
}

void Payload::update(const double dt) {
    m_simTime += dt;

    if (!m_isGrounded) {
        updateForcesOnBody();

        const payloadStates derivatives   = m_calcStatesDerivatives();
        m_eulerIntegration(derivatives, dt);

        // Set new forces/moments to the node of this body
        updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {0.0,0.0,0.0});
        setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::QuaternionD::identity()));
    }

    // Log
    m_writeLog();
}

void Payload::setInitialStates(initialStates& states) {
    m_states.x = states.position[0];
    m_states.y = states.position[1];
    m_states.z = states.position[2];

    m_states.vx = states.position[0];
    m_states.vy = states.position[1];
    m_states.vz = states.position[2];

    updateWorldInfos({m_states.x, m_states.y, m_states.z}, {m_states.vx, m_states.vy, m_states.vz}, {0,0,0});
    setCenterOfMassTransform(grs::TransformD(centerOfMass().position().toOpenglRef(), grs::eulerToQuat(states.orientation[0],states.orientation[1],states.orientation[2]).nedToYup()));
}

void Payload::m_writeLog() const {

    std::ostringstream msg;
    msg << m_simTime << ","
        << m_states.x  << "," << m_states.y  << "," << m_states.z  << ","
        << m_states.vx << "," << m_states.vy << "," << m_states.vz;

    Logger::instance().log(LogType::Payload, msg.str());
}

payloadStates Payload::m_calcStatesDerivatives() const {
    payloadStates derivatives{};

    derivatives.x = m_states.vx;
    derivatives.y = m_states.vy;
    derivatives.z = m_states.vz;

    auto forces = centerOfMass().force();

    derivatives.vx  = forces[0] / m_properties.mass;
    derivatives.vy  = forces[1] / m_properties.mass;
    derivatives.vz  = (forces[2] + m_properties.mass * m_physics.gravity) / m_properties.mass;

    return derivatives;
}

void Payload::m_eulerIntegration(const payloadStates& derivatives, const double dt) {

    m_states.x += derivatives.x * dt;
    m_states.y += derivatives.y * dt;
    m_states.z += derivatives.z * dt;

    m_states.vx += derivatives.vx * dt;
    m_states.vy += derivatives.vy * dt;
    m_states.vz += derivatives.vz * dt;

    // Simulate the ground plane at z = 0
    if (m_states.z >= 0.0 && m_states.vz > 0.0) { // penetrating and moving further down
        m_states.z  = 0.0;
        m_states.vz = 0.0;
    }
}

grs::Vec3d Payload::m_calcAeroForces() const {

    // Drag = -coeff * ||v_rel|| * v_rel; same as ||v_rel||^2 * v_rel/||v_rel||
    const double coeff = 0.5 * m_physics.airDensity * m_properties.cda;
    const grs::Vec3d v_rel = centerOfMass().velocity() - grs::cast<double>(m_physics.wind);

    return -coeff * v_rel.norm() * v_rel;
}
