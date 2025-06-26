/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "payload.h"

Payload::Payload(const worldPhysics& physics, Logger& logger, const payloadProperties& properties, const bool isGrounded)
    : m_physics(physics)
    , m_logger(logger)
    , m_properties(properties)
    , m_isGrounded(isGrounded)
{

}

void Payload::update(const double dt) {

    if (!m_isGrounded) {
        const Vector3<double> externalForcesEarth = calcExternalForce() + m_calcAeroForces();

        const payloadStates derivatives   = m_calcStatesDerivatives(externalForcesEarth);
        m_eulerIntegration(derivatives, dt);

        // Set new forces/moments to the node of this body
        centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
        centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
        centerOfMass->setForce(externalForcesEarth);

        setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::identity()));
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

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});

    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(states.orientation[0],states.orientation[1],states.orientation[2])));
}

void Payload::m_writeLog() {

    payloadLogMessage logMessage = {
        centerOfMass->position(),
        centerOfMass->velocity(),
        externalForces()
    };

    m_logger.writePayloadMessage(logMessage);
}

payloadStates Payload::m_calcStatesDerivatives(Vector3<double> externalForceEarth) const {
    payloadStates derivatives{};

    derivatives.x = m_states.vx;
    derivatives.y = m_states.vy;
    derivatives.z = m_states.vz;

    derivatives.vx  = externalForceEarth[0] / m_properties.mass;
    derivatives.vy  = externalForceEarth[1] / m_properties.mass;
    derivatives.vz  = (externalForceEarth[2] + m_properties.mass * m_physics.gravity) / m_properties.mass;

    return derivatives;
}

void Payload::m_eulerIntegration(const payloadStates& derivatives, const double dt) {

    m_states.x += derivatives.x * dt;
    m_states.y += derivatives.y * dt;
    m_states.z += derivatives.z * dt;

    m_states.vx += derivatives.vx * dt;
    m_states.vy += derivatives.vy * dt;
    m_states.vz += derivatives.vz * dt;

    // Simulate the ground
    if (m_states.z >= -0.001 && derivatives.vz >= -0.001) {
        m_states.z  = 0.0;
        m_states.vz = 0.0;
    }
}

Vector3<double> Payload::m_calcAeroForces() const {

    // Drag = -coeff * ||v_rel|| * v_rel; same as ||v_rel||^2 * v_rel/||v_rel||
    double coeff = 0.5 * m_physics.airDensity * m_properties.cda;
    Vector3<double> v_rel = centerOfMass->velocity() - cast<double>(m_physics.wind);

    return -coeff * v_rel.length() * v_rel;
}
