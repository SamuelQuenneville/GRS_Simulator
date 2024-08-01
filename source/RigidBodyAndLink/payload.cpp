/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "payload.h"

Payload::Payload(worldPhysics& physics, Logger& logger, payloadProperties& properties, bool isGrounded)
    : m_physics(physics)
    , m_logger(logger)
    , m_properties(properties)
    , m_isGrounded(isGrounded)
{

}

void Payload::update(double dt) {

    if (!m_isGrounded) {
        Vector3<double> externalForcesEarth = calcExternalForce();

        payloadStates derivatives   = m_calcStatesDerivatives(externalForcesEarth);
        payloadStates updatedStates = m_eulerIntegration(derivatives, dt);

        // Set new states to this body
        setStates(updatedStates);

        // Set new forces/moments to the node of this body
        centerOfMass->setForce(externalForcesEarth);

        setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::identity()));
    }

    // Log
    m_writeLog();
}

void Payload::setStates(const payloadStates& newStates) {
    m_states = newStates;

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});
}

void Payload::setInitialStates(initialStates& states) {
    m_states.x = states.position.at(0);
    m_states.y = states.position.at(1);
    m_states.z = states.position.at(2);

    m_states.vx = states.position.at(0);
    m_states.vy = states.position.at(1);
    m_states.vz = states.position.at(2);

    centerOfMass->setPosition({m_states.x, m_states.y, m_states.z});
    centerOfMass->setVelocity({m_states.vx, m_states.vy, m_states.vz});

    setTransform(Transform(centerOfMass->position().toOpenglRef(), Quaternion::fromEulerAngles(states.orientation[0],states.orientation[1],states.orientation[2])));
}

void Payload::m_writeLog() {
    payloadLogMessage logMessage{};

    logMessage = {
            centerOfMass->position(),
            centerOfMass->velocity(),
            externalForces()
    };

    m_logger.writePayloadMessage(logMessage);
}

payloadStates Payload::m_calcStatesDerivatives(Vector3<double> externalForceEarth) const {
    payloadStates derivatives{};

    derivatives.vx  = externalForceEarth[0] / m_properties.mass;
    derivatives.vy  = externalForceEarth[1] / m_properties.mass;
    derivatives.vz  = (externalForceEarth[2] + m_properties.mass * m_physics.gravity) / m_properties.mass;

    return derivatives;
}

payloadStates Payload::m_eulerIntegration(payloadStates& derivatives, double dt) const {
    payloadStates updatedStates{};

    updatedStates.vx = m_states.vx + derivatives.vx * dt;
    updatedStates.vy = m_states.vy + derivatives.vy * dt;
    updatedStates.vz = m_states.vz + derivatives.vz * dt;

    updatedStates.x  = m_states.x + updatedStates.vx * dt;
    updatedStates.y  = m_states.y + updatedStates.vy * dt;
    updatedStates.z  = m_states.z + updatedStates.vz * dt;

    // Simulate the ground
    if (updatedStates.z >= -0.001 && derivatives.vz >= -0.001) {
        updatedStates.z  = 0.0;
        updatedStates.vz = 0.0;
    }

    return updatedStates;
}
