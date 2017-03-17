#ifndef SOCKET_TYPES_HPP
#define SOCKET_TYPES_HPP

#include "../joystick/src/joystick.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

struct PickupCommand {
    uint32_t time_ms;
};

sockaddr_un create_udp_addr(const char *path);

int try_create_udp_socket();

void try_bind_path(int sock, sockaddr_un addr);

#endif // SOCKET_TYPES_HPP
