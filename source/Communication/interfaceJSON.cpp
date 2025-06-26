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

void InterfaceJSON::receiveData(std::map<int, std::array<uint16_t, MAX_SERVO_CHANNELS>>& data) {

    std::vector<std::array<uint16_t, MAX_SERVO_CHANNELS>> receivedData(m_instances.size());
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

                const int nbytes = recvfrom(poll_fds[i].fd, &last_pkt, sizeof(servoPacket), 0, reinterpret_cast<sockaddr*>(&src_addr), &addr_len);
                if (nbytes > 0) {
                    m_instances[i].last_json_msg = last_pkt;
                    m_instances[i].last_msg_time = now;
                    m_instances[i].remote_addr = src_addr;
                    m_instances[i].received_this_step = true;

                    // Store valid servo data
                    for (int servo = 0; servo < MAX_SERVO_CHANNELS; servo++) {
                        pwm[servo] = last_pkt.pwm[servo];
                    }
                    receivedData[i] = pwm;
                }
            }
        }

        if (allReceived(m_instances) || std::any_of(m_instances.begin(), m_instances.end(),
            [&](const auto& inst) { return isStale(inst, now); })) {
            for (int i = 0; i < receivedData.size(); ++i) {
                data[i+1] = receivedData[i];
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

    /*
    printf("Roll   : %f, Pitch : %f, Yaw   : %f\n", data.attitude.x(), data.attitude.y(), data.attitude.z());
    printf("Ax     : %f, Ay    : %f, Az    : %f\n", data.acceleration.x(), data.acceleration.y(), data.acceleration.z());
    printf("North  : %f, East  : %f, Down  : %f\n", position.x(), position.y(), position.z());
    printf("V North: %f, V East: %f, V Down: %f\n", velocity.x(), velocity.y(), velocity.z());
    */

    // Form the string
    const std::string message = "\n{\"timestamp\":" + std::to_string(data.timeStampSecond) +
                                ",\"imu\":{" +
                                    "\"gyro\":[" + std::to_string(gyro.x()) + "," + std::to_string(gyro.y()) + "," + std::to_string(gyro.z()) + "]" +
                                    ",\"accel_body\":[" + std::to_string(acceleration.x()) + "," + std::to_string(acceleration.y()) + "," + std::to_string(acceleration.z()) + "]" +
                                "}" +
                                ",\"position\":[" + std::to_string(position.x()) + "," + std::to_string(position.y()) + "," + std::to_string(position.z()) + "]" +
                                ",\"attitude\":[" + std::to_string(attitude.x()) + "," + std::to_string(attitude.y()) + "," + std::to_string(attitude.z()) + "]" +
                                ",\"velocity\":[" + std::to_string(velocity.x()) + "," + std::to_string(velocity.y()) + "," + std::to_string(velocity.z()) + "]" +
                                ",\"airspeed\":"  + std::to_string(data.airspeed) +
                                ",\"rpm\":" + std::to_string(data.rpm) + "}\n";



    sendto(m_instances[uavId-1].socket_fd, message.data(), message.size(), 0,
       reinterpret_cast<sockaddr*>(&m_instances[uavId-1].remote_addr),
       sizeof(m_instances[uavId-1].remote_addr));
}

void InterfaceJSON::setWindvane(const float direction, const float speed) {
    m_windvaneDirection = direction;
    m_windvaneSpeed = speed;
    m_setWindvane = true;
}

bool InterfaceJSON::allReceived(std::vector<instanceState>& instances) {
    return std::all_of(instances.begin(), instances.end(),
                       [](const instanceState& inst) { return inst.received_this_step; });
}

void InterfaceJSON::resetSyncFlags() {
    for (auto& inst : m_instances) {
        inst.received_this_step = false;
    }
}

bool InterfaceJSON::isStale(const instanceState& instance, const std::chrono::steady_clock::time_point now) {
    return (now - instance.last_msg_time) > std::chrono::milliseconds(MAX_WAIT_MS);
}