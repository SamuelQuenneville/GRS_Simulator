/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "interfaceJSON.h"

bool InterfaceJSON::addSocket(const int uavId, const char* fdmAddress, const uint16_t fdmPortIn) {
    auto newSocket = std::make_unique<SocketUDP>();
    if (!newSocket->bind(fdmAddress, fdmPortIn)) {
        LOG_ERROR("Failed to bind socket to " + std::string(fdmAddress) + ":" + std::to_string(fdmPortIn));
        return false;
    }
    m_fcuFrameRate.push_back(0);
    m_fcuFrameCount.push_back(-1);

    instanceState inst;
    inst.id = uavId;
    inst.socket_fd = newSocket->getFileDescriptor();
    inst.last_msg_time = std::chrono::steady_clock::now();
    inst.socket = std::move(newSocket);

    m_instances.push_back(std::move(inst));

    LOG_INFO("Flight Dynamics Model (FDM) at " + std::string(fdmAddress) + ":" + std::to_string(fdmPortIn));
    return true;
}

void InterfaceJSON::receiveData(std::map<int, std::array<uint16_t, MAX_SERVO_CHANNELS>>& pwms, std::map<int, flags>& flagsList) {

    std::vector<std::array<uint16_t, MAX_SERVO_CHANNELS>> receivedPWM(m_instances.size());
    std::vector<flags> receivedFlags(m_instances.size());
    std::vector<pollfd> poll_fds(m_instances.size());

    bool flag = true;
    while (flag) {
        for (size_t i = 0; i < m_instances.size(); ++i) {
            poll_fds[i].fd = m_instances[i].socket_fd;
            poll_fds[i].events = POLLIN;
        }

        const int ret = poll(poll_fds.data(), poll_fds.size(), TIMEOUT_MS);
        if (ret < 0) {
            perror("poll");
            break;
        }

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < poll_fds.size(); ++i) {
            if (poll_fds[i].revents & POLLIN) {
                servoPacket last_pkt{};
                std::array<uint16_t, MAX_SERVO_CHANNELS> pwm{};

                sockaddr_in src_addr{};
                socklen_t addr_len = sizeof(src_addr);

                const size_t nbytes = recvfrom(poll_fds[i].fd, &last_pkt, sizeof(servoPacket), 0, reinterpret_cast<sockaddr*>(&src_addr), &addr_len);
                if (nbytes > 0) {
                    m_instances[i].last_json_msg = last_pkt;
                    m_instances[i].last_msg_time = now;
                    m_instances[i].remote_addr = src_addr;
                    m_instances[i].received_this_step = true;

                    // Store valid servo data
                    for (size_t servo = 0; servo < MAX_SERVO_CHANNELS; servo++) {
                        pwm[servo] = last_pkt.pwm[servo];
                    }
                    receivedPWM[i] = pwm;

                    receivedFlags[i].shouldMove    = last_pkt.shouldMove;
                    receivedFlags[i].endSimulation = last_pkt.endSimulation;
                }
            }
        }

        if (allReceived(m_instances) || std::any_of(m_instances.begin(), m_instances.end(),
            [&](const auto& inst) { return isStale(inst, now); })) {
            for (int i = 0; i < receivedPWM.size(); ++i) {
                pwms[i+1] = receivedPWM[i];
                flagsList[i+1].shouldMove = receivedFlags[i].shouldMove;
                flagsList[i+1].endSimulation = receivedFlags[i].endSimulation;
            }

            flag = false;
            m_fcuOnline = true;
        }

    }
}

void InterfaceJSON::sendData(const int uavId, const sensorData& data) {
    const auto gyro = data.angularVelocity;
    const auto acceleration = data.acceleration;
    const auto position = data.position;
    const auto attitude = data.attitude;
    const auto velocity = data.velocity;

    // Form the string
    const std::string message = "\n{\"timestamp\":" + std::to_string(data.timeStampSecond) +
                                ",\"imu\":{" +
                                    "\"gyro\":[" + std::to_string(gyro[0]) + "," + std::to_string(gyro[1]) + "," + std::to_string(gyro[2]) + "]" +
                                    ",\"accel_body\":[" + std::to_string(acceleration[0]) + "," + std::to_string(acceleration[1]) + "," + std::to_string(acceleration[2]) + "]" +
                                "}" +
                                ",\"position\":[" + std::to_string(position[0]) + "," + std::to_string(position[1]) + "," + std::to_string(position[2]) + "]" +
                                ",\"attitude\":[" + std::to_string(attitude[0]) + "," + std::to_string(attitude[1]) + "," + std::to_string(attitude[2]) + "]" +
                                ",\"velocity\":[" + std::to_string(velocity[0]) + "," + std::to_string(velocity[1]) + "," + std::to_string(velocity[2]) + "]" +
                                ",\"airspeed\":"  + std::to_string(data.airspeed) +
                                ",\"rpm\":" + std::to_string(data.rpm) + "}\n";

    // Log
    std::ostringstream msg;
    msg << std::fixed << std::setprecision(4)
        << data.timeStampSecond << "," << uavId << ","
        << gyro[0] << "," << gyro[1] << "," << gyro[2] << "," << acceleration[0] << "," << acceleration[1] << "," << acceleration[2] << ","
        << position[0] << "," << position[1] << "," << position[2] << "," << attitude[0] << "," << attitude[1] << "," << attitude[2] << ","
        << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," << data.airspeed << "," << data.rpm;
    Logger::instance().log(LogType::SitlOut, msg.str());

    // Send through udp
    sendto(m_instances[uavId-1].socket_fd, message.data(), message.size(), 0,
       reinterpret_cast<sockaddr*>(&m_instances[uavId-1].remote_addr), sizeof(m_instances[uavId-1].remote_addr));
}

void InterfaceJSON::setWindvane(const float direction, const float speed) {
    m_windvaneDirection = direction;
    m_windvaneSpeed = speed;
    m_setWindvane = true;
}

bool InterfaceJSON::allReceived(std::vector<instanceState>& instances) {
    return std::all_of(instances.begin(), instances.end(), [](const instanceState& inst) { return inst.received_this_step; });
}

void InterfaceJSON::resetSyncFlags() {
    for (auto& inst : m_instances) {
        inst.received_this_step = false;
    }
}

bool InterfaceJSON::isStale(const instanceState& instance, const std::chrono::steady_clock::time_point now) {
    return (now - instance.last_msg_time) > std::chrono::milliseconds(MAX_WAIT_MS);
}