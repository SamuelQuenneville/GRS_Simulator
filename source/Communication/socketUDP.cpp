/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "socketUDP.h"

SocketUDP::SocketUDP()
    : m_lastReceiveAddress{}
{
    m_fileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);

    fcntl(m_fileDescriptor, F_SETFD, FD_CLOEXEC);
    fcntl(m_fileDescriptor, F_SETFL, fcntl(m_fileDescriptor, F_GETFL, 0) | O_NONBLOCK);

    constexpr int option = 1;
    setsockopt(m_fileDescriptor, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
}

SocketUDP::~SocketUDP() {
    m_closeSocket();
}

bool SocketUDP::bind(const char* address, const uint16_t port) const {
    sockaddr_in addressIn{};
    m_makeSocketAddress(address, port, addressIn);
    const int result = ::bind(m_fileDescriptor, reinterpret_cast<struct sockaddr*>(&addressIn), sizeof(addressIn));
    return result == 0;
}

void SocketUDP::m_makeSocketAddress(const char* address, const uint16_t port, sockaddr_in& socketAddress) {
    memset(&socketAddress, 0, sizeof(socketAddress));
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_port = htons(port);
    socketAddress.sin_addr.s_addr = inet_addr(address);
}

void SocketUDP::m_closeSocket() {
    if (m_fileDescriptor >= 0) {
        ::close(m_fileDescriptor);
        m_fileDescriptor = -1;
    }
}