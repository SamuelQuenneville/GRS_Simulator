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
    #define MAGIC_32 29569          // From ardupilot JSON standard (32 pwm channels)
#endif

constexpr int TIMEOUT_MS = 10;
constexpr int MAX_WAIT_MS = 50;

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
    double rpm{};
};

struct instanceState {
    int id;
    int socket_fd;
    sockaddr_in remote_addr;
    std::chrono::steady_clock::time_point last_msg_time;
    servoPacket last_json_msg;
    bool received_this_step = false;
    std::unique_ptr<SocketUDP> socket;
};

#endif //JSONINTERFACE_H
