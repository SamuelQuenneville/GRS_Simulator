/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PAYLOAD_H
#define GRS_SIMULATOR_PAYLOAD_H

#include "RigidBodyAndLink/rigidBody.h"
#include "Definitions/payloadDefinitions.h"

class Payload : public RigidBody {

public:
    Payload(worldPhysics& physics, Logger& logger, payloadProperties& properties, bool isGrounded);

    void update(double dt) override;

    [[nodiscard]] payloadStates getStates() const {return m_states;};

    void setInitialStates(initialStates& states) override;

private:
    worldPhysics m_physics;
    Logger& m_logger;

    payloadStates m_states{};
    payloadProperties m_properties;

    bool m_isGrounded; // static body

    Vector3<double> m_calcAeroForces();

    [[nodiscard]] payloadStates m_calcStatesDerivatives(Vector3<double> externalForceEarth) const;
    void m_eulerIntegration(payloadStates& derivatives, double dt);


    void m_writeLog();
};


#endif //GRS_SIMULATOR_PAYLOAD_H
