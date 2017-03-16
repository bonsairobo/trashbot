#ifndef SOCKET_TYPES_HPP
#define SOCKET_TYPES_HPP

#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <cstdint>
#include "../joystick/src/joystick.hpp"

struct JoystickPacket {
    uint32_t time_ms;
    JoystickEvent event;
};

int try_create_udp_socket(
    const char *recv_path,
    const char *send_path,
    sockaddr_un *recv_addr_out,
    sockaddr_un *send_addr_out);

#endif // SOCKET_TYPES_HPP
