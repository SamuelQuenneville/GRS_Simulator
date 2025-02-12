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

class FixedWing final : public RigidBody {

public:
    FixedWing(worldPhysics& physics, Logger& logger, const fixedWingProperties& properties);
    FixedWing(worldPhysics& physics, Logger& logger, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller);
    ~FixedWing();

    [[nodiscard]] int getId() const;

    void update(double dt) override;
    void updateSitl(double dt, const std::array<uint16_t, 16> &pwm);

    [[nodiscard]] fixedWingStates getStates() const {return m_states;};

    void setInitialStates(initialStates& states) override;

    [[nodiscard]] fixedWingCommands getCommands() const {return m_commands;};
    void setCommands(const fixedWingCommands& newCommands) {m_commands = newCommands;};

    [[nodiscard]] fixedWingControlInputs getInputs() const {return m_inputs;};
    void setInputs(const fixedWingControlInputs& newInputs) {m_inputs = newInputs;};

    [[nodiscard]] double airspeed() const;
    [[nodiscard]] double groundspeed() const;

    [[nodiscard]] Matrix3<double> rotationMatrixCBodyEarth() const;

private:
    int m_id;
    static int m_idCounter;

    worldPhysics m_physics;
    Logger& m_logger;

    fixedWingStates m_states{};
    fixedWingProperties m_properties;
    fixedWingControlInputs m_inputs{};
    fixedWingCommands m_commands{};

    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> m_controller = nullptr;

    [[nodiscard]] std::tuple<double, double, double> m_angleOfAttackSideSlip() const;
    [[nodiscard]] Vector3<double> m_calcForceAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;
    [[nodiscard]] Vector3<double> m_calcMomentAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;

    [[nodiscard]] Vector3<double> m_earthToBody(const Vector3<double>& earthVector) const;
    [[nodiscard]] Vector3<double> m_bodyToEarth(const Vector3<double>& bodyVector) const;

    fixedWingStates m_calcStatesDerivatives(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, const Vector3<double>& externalForceEarth);
    void m_eulerIntegration(const fixedWingStates& derivatives, double dt);

    [[nodiscard]] double m_pwmToAngle(const std::string& surface, uint16_t pwm) const;

    void m_writeLog(const Vector3<double>& forceAeroBody, const Vector3<double>& momentAeroBody, const Vector3<double>& tetherForceBody);
};

#endif //GRS_SIMULATOR_FIXEDWING_H
