/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef POWERTRAIN_H
#define POWERTRAIN_H

#include "Definitions/fixedWingDefinitions.h"

double thrust2rpm(float airspeed, float thrustTarget);
double rpm2thrust(float airspeed, float rpmTarget);
float maxThrust(float airspeed);

// ESC, brushed-equivalent DC motor at fixed battery voltage, propeller rotor and RPM sensor
class Powertrain {
public:
    explicit Powertrain(const powertrainProperties& properties) : m_properties(properties) {}

    void update(double dt, double pwm, double airspeed);

    [[nodiscard]] double thrust() const {return m_thrust;}               // N
    [[nodiscard]] double reactionTorque() const;                           // on the airframe about the shaft [N*m]
    [[nodiscard]] double angularMomentum() const;                          // signed, about the shaft [kg*m^2/s]
    [[nodiscard]] double duty() const {return m_duty;}
    [[nodiscard]] double rotorRpm() const;
    [[nodiscard]] double sensedRpm() const {return m_sensedRpm;}
    [[nodiscard]] double current() const {return m_current;}               // A, motor
    [[nodiscard]] double motorTorque() const {return m_motorTorque;}       // N*m, on the rotor

private:
    powertrainProperties m_properties;

    double m_duty        = 0.0;
    double m_omega       = 0.0; // rad/s
    double m_current     = 0.0;
    double m_motorTorque = 0.0;
    double m_thrust      = 0.0;
    double m_filteredRpm = 0.0;
    double m_sensedRpm   = 0.0;
    double m_sensorTimer = 0.0;
};

#endif //POWERTRAIN_H
