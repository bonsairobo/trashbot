#include <iostream>
#include "joystick.hpp"
#include "../../common/socket_types.hpp"
#include <chrono>

using namespace std;

int main(int argc, char **argv) {
    Joystick js;
    if (!js.isFound()) {
        cerr << "ERROR: No joystick found." << endl;
        return 1;
    }

    // This process is the sender.
    struct sockaddr_un recv_addr, send_addr;
    int sock = try_create_udp_socket(
        "/tmp/motion_controller_endpoint",
        "/tmp/joystick_endpoint",
        &recv_addr,
        &send_addr);
    // If the path already exists, must unlink before rebinding.
    if (access(send_addr.sun_path, F_OK) == 0)
        unlink(send_addr.sun_path);
    if (bind(sock, (sockaddr*)&send_addr, sizeof(send_addr)) == -1) {
        perror("bind");
        exit(-1);
    }

    while (true) {
        usleep(1000);
        JoystickPacket packet;
        while (js.sample(&packet.event)) {
            if (packet.event.isButton()) {
                cout << "BUTTON PRESS" << endl;
                auto now = chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                packet.time_ms = chrono::duration_cast<chrono::milliseconds>(
                    duration).count();
                sendto(
                    sock,
                    &packet,
                    sizeof(packet),
                    0,
                    (sockaddr*)&recv_addr,
                    sizeof(recv_addr));
            }
        }
    }

    return 0;
}
