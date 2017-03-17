#include <iostream>
#include <cmath>
#include <queue>
#include "../common/socket_types.hpp"
#include <cassert>

using namespace std;

int main(int argc, char **argv) {
    sockaddr_un recv_addr = create_udp_addr("/tmp/motion_controller_endpoint");
    sockaddr_un send_addr = create_udp_addr("/tmp/joystick_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, recv_addr);

    while (true) {
        JoystickEvent event;
        socklen_t len = sizeof(send_addr);
        ssize_t bytes_read = recvfrom(
            sock,
            &event,
            sizeof(event),
            0,
            (sockaddr*)&send_addr,
            &len);
        if (bytes_read < 0) {
            perror("recvfrom");
            return 1;
        } else if (bytes_read != sizeof(event)) {
            usleep(1000);
            continue;
        }

        // TODO: Process event.
        cout << "Packet received from time: " << event.time << endl;
    }

    return 0;
}
