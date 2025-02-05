/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef INTERFACEJSON_H
#define INTERFACEJSON_H

#include <iostream>
#include <string>
#include <array>

#include "socketUDP.h"
#include "Definitions/jsonInterface.h"
#include "RigidBodyAndLink/rigidBody.h"

#define CONNECTION_TIMEOUT_COUNT_MAX 10

class InterfaceJSON {

public:
    InterfaceJSON() = default;
    ~InterfaceJSON() = default;

    bool addSocket(const char* fdmAddress, uint16_t fdmPortIn);
    bool receiveData(std::vector<std::array<uint16_t, MAX_SERVO_CHANNELS>>& data);

    void sendData(const SocketUDP &socket, const sensorData &data);
    void setWindvane(float direction, float speed);

    std::vector<SocketUDP*>& getSockets() {return m_sockets;};

    bool fcuOnline = false;

private:
    std::vector<SocketUDP*> m_sockets;
    const uint16_t m_magic = 18458;

    const char* m_fdmAddress{};
    uint16_t m_fdmPortIn{};

    int m_connectionTimeoutCount = 0;
    std::vector<uint16_t> m_fcuFrameRate;
    std::vector<uint32_t> m_fcuFrameCount;

    float m_windvaneDirection = 0.0;    // Radians clockwise to the front (0 is head to wind)
    float m_windvaneSpeed = 0.0;        // m/s
    bool m_setWindvane = false;
};

#endif //INTERFACEJSON_H
