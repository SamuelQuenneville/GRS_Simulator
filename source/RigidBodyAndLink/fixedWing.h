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
#include "powertrain.h"

class FixedWing final : public RigidBody {

public:
    FixedWing(const worldPhysics& physics, Logger& logger, const fixedWingProperties& properties);
    FixedWing(const worldPhysics& physics, Logger& logger, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller);
    ~FixedWing() override;

    [[nodiscard]] int getId() const;

    void update(double dt) override;
    void updateSitl(double dt, const std::array<uint16_t, 16>& pwm, bool shouldMove);

    [[nodiscard]] fixedWingStates getStates() const {return m_states;};

    void setInitialStates(initialStates& states) override;

    [[nodiscard]] fixedWingCommands getCommands() const {return m_commands;};
    void setCommands(const fixedWingCommands& newCommands) {m_commands = newCommands;};

    [[nodiscard]] fixedWingControlInputs getInputs() const {return m_inputs;};
    void setInputs(const fixedWingControlInputs& newInputs) {m_inputs = newInputs;};

    void setLaunchTriggered(const bool launchTriggered) {m_launchTriggered = launchTriggered;};
    void setInLaunch(const bool inLaunch) {m_inLaunch = inLaunch;};
    void setLauncherAccel(const float newAccel) {m_launcherAccel = newAccel;};

    [[nodiscard]] double getRpm() const {return m_rpm;};

    [[nodiscard]] double airspeed() const;
    [[nodiscard]] double groundspeed() const;

    void updateRotationMatrixCBodyEarth();

    enum controlAxe {
        AILERON,
        ELEVATOR,
        RUDDER,
        THROTTLE,
    };

    float TIME = 0.0f;

private:
    int m_id;
    static int m_idCounter;

    worldPhysics m_physics;
    Logger& m_logger;

    fixedWingStates m_states{};
    fixedWingProperties m_properties;
    fixedWingControlInputs m_inputs{};
    fixedWingCommands m_commands{};

    bool m_sitl = false;

    bool m_launchTriggered = false;
    bool m_inLaunch = false;
    float m_launcherAccel = 0.0;

    bool m_flightLaunchTriggered = false;
    bool m_inFlightLaunch = false;

    double m_rpm = 0.0;
    double m_rpmTau = 0.12;

    Matrix3<double> m_rotationMatrixCBodyEarth = Matrix3<double>::eye();

    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> m_controller = nullptr;

    [[nodiscard]] std::tuple<double, double, double> m_angleOfAttackSideSlip() const;
    [[nodiscard]] Vector3<double> m_calcForceAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;
    [[nodiscard]] Vector3<double> m_calcMomentAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;

    [[nodiscard]] Vector3<double> m_earthToBody(const Vector3<double>& earthVector);
    [[nodiscard]] Vector3<double> m_bodyToEarth(const Vector3<double>& bodyVector);

    fixedWingStates m_calcStatesDerivatives(Vector3<double> forceAeroBody, Vector3<double> momentAeroBody, const Vector3<double>& externalForceEarth);
    void m_eulerIntegration(const fixedWingStates& derivatives, double dt);

    [[nodiscard]] double m_pwmToControl(controlAxe control, uint16_t pwm) const;
    void m_updateRpmSensor(double dt, double throttleInput);

    void m_writeLog(const Vector3<double>& forceAeroBody, const Vector3<double>& momentAeroBody, const Vector3<double>& tetherForceBody);
};

#endif //GRS_SIMULATOR_FIXEDWING_H
