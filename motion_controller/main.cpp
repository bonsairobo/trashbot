#include <iostream>
#include <cmath>
#include <queue>
#include <cassert>
#include "../common/socket_types.hpp"

using namespace std;

int main(int argc, char **argv) {
    // This process is the receiver.
    struct sockaddr_un recv_addr, send_addr;
    int sock = try_create_udp_socket(
        "/tmp/motion_controller_endpoint",
        "/tmp/joystick_endpoint",
        &recv_addr,
        &send_addr);
    if (bind(sock, (sockaddr*)&recv_addr, sizeof(recv_addr)) == -1) {
        cerr << "ERROR: could not bind socket" << endl;
        exit(-1);
    }

    while (true) {
        JoystickPacket packet;
        size_t bytes_read = recvfrom(
            sock,
            &packet,
            sizeof(packet),
            0,
            (sockaddr*)&send_addr,
            (socklen_t*) sizeof(send_addr));
        if (bytes_read < 0) {
            cerr << "ERROR: could not read from socket" << endl;
            return 1;
        } else if (bytes_read != sizeof(packet)) {
            continue;
        }

        // TODO: Process event.
        cout << "Packet received from time: " << packet.time_ms << endl;
    }

    return 0;
}
