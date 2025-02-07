/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef JSONINTERFACE_H
#define JSONINTERFACE_H

#pragma once
#include "Mathematics/vector3.h"

#define MAX_SERVO_CHANNELS 16

#if (MAX_SERVO_CHANNELS == 16)
    #define MAGIC_16 18458          // From ardupilot JSON standard (16 pwm channels)
#endif

// The servo packet received from ArduPilot SITL. Defined in SIM_JSON.h.
struct servoPacket {
    uint16_t magic;
    uint16_t frameRate;
    uint32_t frameCount;
    uint16_t pwm[MAX_SERVO_CHANNELS]; // pwm [aileron , elevator , throttle , rudder , ...]
};

struct sensorData {
    double timeStampSecond{};
    Vector3<double> angularVelocity;
    Vector3<double> acceleration;
    Vector3<double> position;
    Vector3<double> attitude;
    Vector3<double> velocity;
    double airspeed{};
};

#endif //JSONINTERFACE_H
