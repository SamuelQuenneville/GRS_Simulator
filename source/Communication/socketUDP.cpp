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

ssize_t SocketUDP::sendTo(const void* packet, const size_t size, const char* address, const uint16_t port) const {
    sockaddr_in destinationAddress{};
    m_makeSocketAddress(address, port, destinationAddress);
    return ::sendto(m_fileDescriptor, packet, size, 0, reinterpret_cast<sockaddr *>(&destinationAddress), sizeof(destinationAddress));
}

ssize_t SocketUDP::receive(void* packet, const size_t size, const uint32_t timeoutMilliSeconds) {
    if (!pollIn(timeoutMilliSeconds)) {
        return -1;
    }

    socklen_t addressLength = sizeof(m_lastReceiveAddress);
    const ssize_t receiveLength = ::recvfrom(m_fileDescriptor, packet, size, MSG_DONTWAIT, reinterpret_cast<sockaddr *>(&m_lastReceiveAddress), &addressLength);
    return receiveLength;
}

void SocketUDP::lastReceivedAddress(const char*& ipAddress, uint16_t& port) const {
    ipAddress = inet_ntoa(m_lastReceiveAddress.sin_addr);
    port = ntohs(m_lastReceiveAddress.sin_port);
}

bool SocketUDP::pollIn(const uint32_t timeoutMilliSeconds) const {
    fd_set readFileDescriptors;
    timeval timeout{};
    FD_ZERO(&readFileDescriptors);
    FD_SET(m_fileDescriptor, &readFileDescriptors);

    timeout.tv_sec = timeoutMilliSeconds / 1000;
    timeout.tv_usec = (timeoutMilliSeconds % 1000) * 1000;

    if (select(m_fileDescriptor + 1, &readFileDescriptors, nullptr, nullptr, &timeout) != 1) {
        return false;
    }
    return true;
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