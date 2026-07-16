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
#include "Definitions/renderObjectDefinitions.h"
#include "powertrain.h"

#define MAX_SERVO_CHANNELS 16

class FixedWing final : public RigidBody {

public:
    FixedWing(const worldPhysics& physics, const fixedWingProperties& properties);
    FixedWing(const worldPhysics& physics, const fixedWingProperties& properties, std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>>&& controller);
    ~FixedWing() override;

    [[nodiscard]] int getId() const;

    void update(double dt, const WindEnvironment& windEnv) override;
    void updateSitl(double dt, const WindEnvironment& windEnv, const std::array<uint16_t, MAX_SERVO_CHANNELS>& pwm, bool shouldMove);

    [[nodiscard]] fixedWingStates getStates() const {return m_states;};

    void setInitialStates(initialStates& states) override;

    [[nodiscard]] fixedWingCommands getCommands() const {return m_commands;};
    void setCommands(const fixedWingCommands& newCommands) {m_commands = newCommands;};

    [[nodiscard]] fixedWingControlInputs getInputs() const {return m_inputs;};
    void setInputs(const fixedWingControlInputs& newInputs) {m_inputs = newInputs;};

    void setLaunchTriggered(const bool launchTriggered) {m_launchTriggered = launchTriggered;};
    void setInLaunch(const bool inLaunch) {m_inLaunch = inLaunch;};
    void setLauncherAccel(const float newAccel) {m_launcherAccel = newAccel;};

    void setInFlightLaunch() {m_inFlightLaunch = true;};

    [[nodiscard]] double getRpm() const {return m_rpm;};

    [[nodiscard]] double airspeed() const;
    [[nodiscard]] double groundspeed() const;

    void updateRotationMatrixCBodyEarth();

    [[nodiscard]] bodyRenderData packRenderData() const;

    enum controlAxis {
        AILERON,
        ELEVATOR,
        RUDDER,
        THROTTLE,
    };

private:
    int m_id;
    static int m_idCounter;

    double m_simTime = 0.0;

    worldPhysics m_physics;
    grs::Vec3d m_windEarth = grs::Vec3d::zeros();
    grs::Vec3d m_windBody = grs::Vec3d::zeros();

    fixedWingStates m_states{};
    fixedWingProperties m_properties;
    fixedWingControlInputs m_inputs{};
    fixedWingCommands m_commands{};

    bool m_sitl = false;

    double m_launcherPitch = 0.0;
    double m_launcherYaw = 0.0;

    bool m_launchTriggered = false;
    bool m_inLaunch = false;
    float m_launcherAccel = 0.0f;

    bool m_flightLaunchTriggered = false;
    bool m_inFlightLaunch = false;

    double m_rpm = 0.0;
    double m_rpmTau = 0.12;

    std::unique_ptr<LowLevelController<fixedWingControlInputs,fixedWingStates,fixedWingCommands>> m_controller = nullptr;

    [[nodiscard]] std::tuple<double, double, double> m_angleOfAttackSideSlip() const;
    [[nodiscard]] grs::Vec3d m_calcForceAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;
    [[nodiscard]] grs::Vec3d m_calcMomentAeroBodyFrame(double airspeed, double alphaRadian, double betaRadian) const;

    [[nodiscard]] grs::Vec3d m_earthToBody(const grs::Vec3d& earthVector) const;
    [[nodiscard]] grs::Vec3d m_bodyToEarth(const grs::Vec3d& bodyVector) const;

    void m_integrateCatapult(double dt);

    fixedWingStates m_calcStatesDerivatives();
    void m_eulerIntegration(const fixedWingStates& derivatives, double dt);

    [[nodiscard]] double m_pwmToControl(controlAxis control, uint16_t pwm) const;
    void m_updateRpmSensor(double dt, double throttleInput);

    void m_writeLog();
};

#endif //GRS_SIMULATOR_FIXEDWING_H
