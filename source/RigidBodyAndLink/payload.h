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
#include "Definitions/renderObjectDefinitions.h"

class Payload final : public RigidBody {

public:
    Payload(const worldPhysics& physics, const payloadProperties& properties, bool isGrounded);

    void update(double dt, const WindEnvironment& windEnv) override;

    [[nodiscard]] int getId() const;
    void setId(int id);

    [[nodiscard]] payloadStates getStates() const {return m_states;};

    void setInitialStates(initialStates& states) override;

    [[nodiscard]] bodyRenderData packRenderData() const;

private:
    int m_id;
    double m_simTime = 0.0;

    worldPhysics m_physics;
    grs::Vec3d m_wind = grs::Vec3d::zeros();

    payloadStates m_states{};
    payloadProperties m_properties;

    bool m_isGrounded; // static body

    [[nodiscard]] grs::Vec3d m_calcAeroForces() const;

    [[nodiscard]] payloadStates m_calcStatesDerivatives() const;
    void m_eulerIntegration(const payloadStates& derivatives, double dt);


    void m_writeLog() const;
};


#endif //GRS_SIMULATOR_PAYLOAD_H
