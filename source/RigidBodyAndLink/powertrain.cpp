/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "powertrain.h"

#include <algorithm>
#include <cmath>

namespace {
    // thrust = (-X*u^2 - Y*u + Z) * rho * rps^2 * D^4, u = airspeed / (rps*D).
    // With n in rpm and phi = 60*airspeed/D (so u = phi/n):
    // thrust = psi * (Z*n^2 - Y*phi*n - X*phi^2), psi = rho*D^4/3600.
    constexpr double kAirDensity = 1.225;            // [kg/m^3]
    constexpr double kPropDiameter = 16.0 * 0.0254;  // [m]
    constexpr double kThrustCoeffX = 0.1588;
    constexpr double kThrustCoeffY = 0.0106;
    constexpr double kThrustCoeffZ = 0.0757;
    constexpr double kPsi = kAirDensity * kPropDiameter * kPropDiameter * kPropDiameter * kPropDiameter / 3600.0;
    constexpr double kMaxRpm = 9000.0;

    // Power coefficient CP(J) = c0 + c1 J + c2 J^2, J = airspeed / (rps*D), P = CP rho n^3 D^5.
    // Fitted on the wind tunnel data of the APC 16x8E, as the thrust coefficients above.
    constexpr double kPowerCoeff0 = 0.0285;
    constexpr double kPowerCoeff1 = 0.0606;
    constexpr double kPowerCoeff2 = -0.1450;
    constexpr double kTwoPi = 6.283185307179586;
    constexpr double kPropInertia = 3.4e-4;    // Propeller + motor rotating part about the shaft [kg*m^2], measured
    constexpr double kPropSpinDirection = 1.0; // +1: clockwise seen from behind, -1: counter-clockwise

    // Aerodynamic torque of the propeller about the shaft, CP clamped at 0 (no windmilling) [N*m]
    double propTorque(const double airspeed, const double rpm) {
        if (!(rpm > 0.0)) {
            return 0.0;
        }
        const double n = rpm / 60.0;
        const double J = airspeed / (n * kPropDiameter);
        const double cp = std::max(kPowerCoeff0 + kPowerCoeff1 * J + kPowerCoeff2 * J * J, 0.0);
        return cp / kTwoPi * kAirDensity * n * n * std::pow(kPropDiameter, 5);
    }

    double firstOrderLag(const double y, const double target, const double dt, const double tau) {
        return tau > 0.0 ? y + (target - y) * (1.0 - std::exp(-dt / tau)) : target;
    }
}

double thrust2rpm(const float airspeed, const float thrustTarget) {
    if (!std::isfinite(thrustTarget)) {
        return 0.0;
    }

    const double v = std::isfinite(airspeed) ? std::max(static_cast<double>(airspeed), 0.0) : 0.0;
    const double phi = 60.0 * v / kPropDiameter;

    // Positive root of Z*n^2 - Y*phi*n - (X*phi^2 + T/psi) = 0.
    const double c = kThrustCoeffX * phi * phi + std::max(static_cast<double>(thrustTarget), 0.0) / kPsi;
    const double rpm = (kThrustCoeffY * phi + std::sqrt(kThrustCoeffY * kThrustCoeffY * phi * phi + 4.0 * kThrustCoeffZ * c)) / (2.0 * kThrustCoeffZ);

    return std::clamp(rpm, 0.0, kMaxRpm) / kMaxRpm;
}

double rpm2thrust(const float airspeed, const float rpmTarget) {
    const double phi = 60.0 * airspeed / kPropDiameter;
    const double n = rpmTarget;
    return kPsi * (kThrustCoeffZ * n * n - kThrustCoeffY * phi * n - kThrustCoeffX * phi * phi);
}

float maxThrust(const float airspeed) {
    return static_cast<float>(rpm2thrust(airspeed, kMaxRpm));
}

double Powertrain::rotorRpm() const {
    return m_omega * 60.0 / kTwoPi;
}

double Powertrain::reactionTorque() const {
    return -kPropSpinDirection * m_motorTorque;
}

double Powertrain::angularMomentum() const {
    return kPropSpinDirection * kPropInertia * m_omega;
}

void Powertrain::update(const double dt, const double pwm, const double airspeed) {
    const auto& p = m_properties;

    const double dutyTarget = std::clamp((pwm - p.escPwmMin) / (p.escPwmMax - p.escPwmMin), 0.0, 1.0);
    m_duty = firstOrderLag(m_duty, dutyTarget, dt, p.escTau);

    // Back-EMF constant [V*s/rad], equal to the torque constant [N*m/A].
    // Current is quasi-static (L/R << dt); the ESC does not brake regeneratively.
    const double ke = 60.0 / (kTwoPi * p.motorKv);
    m_current = std::max((m_duty * p.batteryVoltage - ke * m_omega) / p.motorResistance, 0.0);
    m_motorTorque = ke * (m_current - p.motorNoLoadCurrent);
    if (m_omega <= 0.0) {
        m_motorTorque = std::max(m_motorTorque, 0.0);
    }

    const double v = std::max(airspeed, 0.0);
    m_omega = std::max(m_omega + dt * (m_motorTorque - propTorque(v, rotorRpm())) / kPropInertia, 0.0);

    const double rpm = rotorRpm();
    m_thrust = rpm < 10.0 ? 0.0 : std::max(rpm2thrust(static_cast<float>(v), static_cast<float>(rpm)), 0.0);

    m_filteredRpm = firstOrderLag(m_filteredRpm, rpm, dt, p.rpmSensorTau);
    if (p.rpmSensorRateHz <= 0.0) {
        m_sensedRpm = m_filteredRpm;
        return;
    }
    const double period = 1.0 / p.rpmSensorRateHz;
    m_sensorTimer += dt;
    if (m_sensorTimer >= period) {
        m_sensorTimer = std::fmod(m_sensorTimer, period);
        m_sensedRpm = m_filteredRpm;
    }
}
