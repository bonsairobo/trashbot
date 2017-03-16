#include <iostream>
#include <cmath>
#include <queue>
#include <cassert>
#include "../common/socket_types.hpp"

using namespace std;

int main(int argc, char **argv) {
    struct sockaddr_un recv_addr, send_addr;
    int sock = try_create_udp_socket(
        "/tmp/motion_controller_endpoint",
        "/tmp/joystick_endpoint",
        &recv_addr,
        &send_addr);
    // This process is the receiver.
    try_bind_path(sock, recv_addr);

    while (true) {
        JoystickPacket packet;
        socklen_t len = sizeof(send_addr);
        ssize_t bytes_read = recvfrom(
            sock,
            &packet,
            sizeof(packet),
            0,
            (sockaddr*)&send_addr,
            &len);
        if (bytes_read < 0) {
            perror("recvfrom");
            return 1;
        } else if (bytes_read != sizeof(packet)) {
            usleep(1000);
            continue;
        }

        // TODO: Process event.
        cout << "Packet received from time: " << packet.time_ms << endl;
    }

    return 0;
}
