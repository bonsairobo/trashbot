#include "socket_types.hpp"
#include <iostream>

using namespace std;

CodePacket::CodePacket(PacketType type): type(type) {}

sockaddr_un create_udp_addr(const char *path) {
    sockaddr_un addr;
    memset(&addr, 0, sizeof(sockaddr_un));
    addr.sun_family = AF_UNIX;
    // Copy one byte fewer to ensure there is a null-terminator.
    strncpy(addr.sun_path, path, sizeof(addr.sun_path)-1);
    return addr;
}

int try_create_udp_socket() {
    int sock;
    if ((sock = socket(AF_UNIX, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }
    return sock;
}

void try_bind_path(int sock, sockaddr_un addr) {
    // If the path already exists, must unlink before rebinding.
    if (access(addr.sun_path, F_OK) == 0)
        unlink(addr.sun_path);
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("bind");
        exit(1);
    }
}
