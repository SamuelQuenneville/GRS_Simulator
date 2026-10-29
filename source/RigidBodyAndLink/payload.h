/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PAYLOAD_H
#define GRS_SIMULATOR_PAYLOAD_H

#include <algorithm>

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
    // Returns true when the ground plane constrained the body this step
    bool m_eulerIntegration(const payloadStates& derivatives, double dt);
    // Coulomb friction while resting on the ground: holds the body while the
    // horizontal force stays below frictionStatic * normal force, else slides
    // against frictionKinetic * normal force. Modifies the horizontal
    // accelerations (and velocity when it sticks).
    void m_applyGroundFriction(payloadStates& derivatives, double dt);

    // Payload has no attitude (identity), so body frame == NED: f_b = a_NED - [0 0 g]
    [[nodiscard]] grs::Vec3d m_calcSpecificForceBody(const grs::Vec3d& accelNed) const;


    void m_writeLog() const;
};


#endif //GRS_SIMULATOR_PAYLOAD_H
