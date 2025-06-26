/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef SOCKETUDP_H
#define SOCKETUDP_H

#pragma once

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

#include "Log/programLogger.h"

class SocketUDP {

public:
    SocketUDP();
    ~SocketUDP();

    bool bind(const char* address, uint16_t port) const;
    [[nodiscard]] int getFileDescriptor() const {return m_fileDescriptor;};

private:
    int m_fileDescriptor;

    sockaddr_in m_lastReceiveAddress;

    static void m_makeSocketAddress(const char* address, uint16_t port, sockaddr_in& socketAddress);
    void m_closeSocket();
};

#endif //SOCKETUDP_H
