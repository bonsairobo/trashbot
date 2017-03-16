#include "socket_types.hpp"
#include <iostream>

using namespace std;

int try_create_udp_socket(
    const char *recv_path,
    const char *send_path,
    sockaddr_un *recv_addr_out,
    sockaddr_un *send_addr_out)
{
    memset(recv_addr_out, 0, sizeof(sockaddr_un));
    memset(send_addr_out, 0, sizeof(sockaddr_un));
    recv_addr_out->sun_family = AF_UNIX;
    send_addr_out->sun_family = AF_UNIX;

    // Copy one byte fewer to ensure there is a null-terminator.
    strncpy(recv_addr_out->sun_path, recv_path, sizeof(sockaddr_un)-1);
    strncpy(send_addr_out->sun_path, send_path, sizeof(sockaddr_un)-1);

    int sock;
    if ((sock = socket(AF_UNIX, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(-1);
    }

    return sock;
}

void try_bind_path(int sock, sockaddr_un addr) {
    // If the path already exists, must unlink before rebinding.
    if (access(addr.sun_path, F_OK) == 0)
        unlink(addr.sun_path);
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("bind");
        exit(-1);
    }
}
