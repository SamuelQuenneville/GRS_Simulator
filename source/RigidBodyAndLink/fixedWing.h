/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FIXEDWING_H
#define GRS_SIMULATOR_FIXEDWING_H

#include "RigidBodyAndLink/rigidBody.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Control/lowLevelController.h"


class FixedWing : public RigidBody {

public:
    FixedWing(worldPhysics& physics, fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller);

    [[nodiscard]] int getId() const;
    [[nodiscard]] rigidBodyType getType() const;

    void update(double dt) override;

    [[nodiscard]] fixedWingStates getStates() const {return m_states;};
    virtual void setStates(const fixedWingStates& newStates) {m_states = newStates;};

    void setInitialStates(initialStates& states) override;

    [[nodiscard]] fixedWingCommands getCommands() const {return m_commands;};
    void setCommands(const fixedWingCommands& newCommands) {m_commands = newCommands;};

    [[nodiscard]] double airspeed() const;

    [[nodiscard]] Matrix3<double> rotationMatrixCBodyEarth() const;

private:
    int m_id;
    static int m_idCounter;
    rigidBodyType m_type;

    worldPhysics m_physics;

    fixedWingStates m_states{};
    fixedWingProperties m_properties;
    fixedWingCommands m_commands{};

    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> m_controller;

    [[nodiscard]] Vector3<double> m_calcForceAeroBodyFrame(const fixedWingControlInputs& inputs) const;
    [[nodiscard]] Vector3<double> m_calcMomentAeroBodyFrame(const fixedWingControlInputs& inputs) const;

    [[nodiscard]] Vector3<double> m_earthToBody(const Vector3<double>& earthVector) const;
    [[nodiscard]] Vector3<double> m_bodyToEarth(const Vector3<double>& bodyVector) const;

    fixedWingStates m_calcStatesDerivatives(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, Vector3<double> externalForceEarth);
    fixedWingStates m_eulerIntegration(fixedWingStates& derivatives, double dt) const;

};

#endif //GRS_SIMULATOR_FIXEDWING_H
