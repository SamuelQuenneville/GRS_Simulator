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
#include <poll.h>
#include <thread>

#include "socketUDP.h"
#include "Definitions/jsonInterface.h"
#include "RigidBodyAndLink/rigidBody.h"

class InterfaceJSON {

public:
    InterfaceJSON() = default;
    ~InterfaceJSON() = default;

    bool addSocket(int uavId, const char* fdmAddress, uint16_t fdmPortIn);
    void receiveData(std::map<int, std::array<uint16_t, MAX_SERVO_CHANNELS>>& pwms, std::map<int, flags>& flagsList);
    void sendData(int uavId, const sensorData &data);
    void setWindvane(float direction, float speed);

    static bool allReceived(std::vector<instanceState>& instances);
    void resetSyncFlags();
    static bool isStale(const instanceState& instance, std::chrono::steady_clock::time_point now);

    [[nodiscard]] bool fcuOnline() const {return m_fcuOnline;};

private:
    std::vector<instanceState> m_instances;
    std::vector<std::unique_ptr<SocketUDP>> m_sockets;

    bool m_fcuOnline = false;

    std::vector<uint16_t> m_fcuFrameRate;
    std::vector<uint32_t> m_fcuFrameCount;

    float m_windvaneDirection = 0.0;        // Radians clockwise to the front (0 is head to wind)
    float m_windvaneSpeed = 0.0;            // m/s
    bool m_setWindvane = false;
};

#endif //INTERFACEJSON_H