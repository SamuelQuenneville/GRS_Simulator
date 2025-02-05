/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "interfaceJSON.h"

bool InterfaceJSON::addSocket(const char* fdmAddress, const uint16_t fdmPortIn) {
    const auto newSocket = new SocketUDP();
    if (!newSocket->bind(fdmAddress, fdmPortIn)) {
        std::cout << "[UDP SERVER] " << "Failed to bind to " << fdmAddress << ":" << fdmPortIn << std::endl;
        delete newSocket;
        return false;
    }
    m_sockets.push_back(newSocket);

    m_fdmAddress = fdmAddress;
    m_fdmPortIn = fdmPortIn;

    m_fcuFrameRate.push_back(0);
    m_fcuFrameCount.push_back(-1);

    std::cout << "[UDP SERVER] " << "Flight Dynamics Model (FDM) at " << fdmAddress << ":" << fdmPortIn << std::endl;
    return true;
}

bool InterfaceJSON::receiveData(std::vector<std::array<uint16_t, MAX_SERVO_CHANNELS>>& data) {

    if (m_sockets.empty()) {
        return false;
    }

    std::vector<std::array<uint16_t, MAX_SERVO_CHANNELS>> receivedData(m_sockets.size());

    bool allReceived = true;

    for (size_t i = 0; i < m_sockets.size(); ++i) {
        servoPacket pkt{};
        std::array<uint16_t, MAX_SERVO_CHANNELS> pwm{};

        ssize_t receiveSize = m_sockets[i]->receive(&pkt, sizeof(servoPacket),10);

        // drain the socket in the case we're backed up
        int counter = 0;
        while (true) {
            servoPacket last_pkt{};
            const auto recvSize_last = m_sockets[i]->receive(&last_pkt, sizeof(servoPacket), 0ul);
            if (recvSize_last == -1) {
                break;
            }
            counter++;
            pkt = last_pkt;
            receiveSize = recvSize_last;
        }

        if (counter > 0) {
            std::cout << "[UDP SERVER] Drained " << counter << " packets from socket " << i << std::endl;
        }

        if (receiveSize == -1) {
            allReceived = false; // Mark as failed if any socket did not receive data
            continue;
        }

        if (m_magic != pkt.magic) {
            std::cout << "[UDP SERVER] Incorrect protocol magic " << pkt.magic << " should be " << m_magic << std::endl;
            allReceived = false;
            continue;
        }

        // check frame rate and frame order
        m_fcuFrameRate.at(i) = pkt.frameRate;
        if (pkt.frameCount < m_fcuFrameCount.at(i)) {
            std::cout << "[UDP SERVER] ArduPilot controller has reset\n";
        }
        else if (pkt.frameCount == m_fcuFrameCount.at(i)) {
            // received duplicate frame, skip
            std::cout << "[UDP SERVER] Duplicate input frame count" << m_fcuFrameCount.at(i) << "\n";
            continue;
        }
        else if (pkt.frameCount != m_fcuFrameCount.at(i) + 1 && fcuOnline) {
            // missed frames, warn only
            std::cout << "[UDP SERVER] Missed " << pkt.frameCount - m_fcuFrameCount.at(i) << " input frames\n";
        }
        m_fcuFrameCount.at(i) = pkt.frameCount;

        // Store valid servo data
        for (int servo = 0; servo < MAX_SERVO_CHANNELS; servo++) {
            pwm.at(servo) = pkt.pwm[servo];
        }
        receivedData.at(i) = pwm;

        //std::cout << "[UDP SERVER] Received data " << i << " : " << pkt.frameCount << " : " << pkt.frameRate << "\n";
    }

    if (!fcuOnline && allReceived) {
        fcuOnline = true;
        std::cout << "[UDP SERVER] Connected to all ArduPilot FCU" << std::endl;
    }

    // If all sockets received at least one message, add to output
    if (allReceived) {
        data.insert(data.end(), receivedData.begin(), receivedData.end());
    }

    return allReceived;
}

void InterfaceJSON::sendData(const SocketUDP& socket, const sensorData& data) {
    const auto gyro = data.angularVelocity;
    const auto acceleration = data.acceleration;
    const auto position = data.position;
    const auto attitude = data.attitude;
    const auto velocity = data.velocity;

    // Form the string
    const std::string message = "\n{\"timestamp\":" + std::to_string(data.timeStampSecond) +
                                ",\"imu\":{" +
                                    "\"gyro\":[" + std::to_string(gyro.x()) + "," + std::to_string(gyro.y()) + "," + std::to_string(gyro.z()) + "]" +
                                    ",\"accel_body\":[" + std::to_string(acceleration.x()) + "," + std::to_string(acceleration.y()) + "," + std::to_string(acceleration.z()) + "]" +
                                "}" +
                                ",\"position\":[" + std::to_string(position.x()) + "," + std::to_string(position.y()) + "," + std::to_string(position.z()) + "]" +
                                ",\"attitude\":[" + std::to_string(attitude.x()) + "," + std::to_string(attitude.y()) + "," + std::to_string(attitude.z()) + "]" +
                                ",\"velocity\":[" + std::to_string(velocity.x()) + "," + std::to_string(velocity.y()) + "," + std::to_string(velocity.z()) + "]" +
                                ",\"airspeed\":"  + std::to_string(data.airspeed) + "}\n";

    const char* fcuAddress{};
    uint16_t fcuPortOut{};

    std::cout << message << std::endl;

    socket.lastReceivedAddress(fcuAddress, fcuPortOut);
    [[maybe_unused]] auto bytesSent = socket.sendTo(message.c_str(), message.size(), fcuAddress, fcuPortOut);
}

void InterfaceJSON::setWindvane(const float direction, const float speed) {
    m_windvaneDirection = direction;
    m_windvaneSpeed = speed;
    m_setWindvane = true;
}
