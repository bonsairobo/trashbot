#include <iostream>
#include <cmath>
#include <queue>
#include "../common/socket_types.hpp"
#include <cassert>

using namespace std;

float clamp(float x) {
    if (x < -1.0)
        return -1.0;
    if (x > 1.0)
        return 1.0;
    return x;
}

int main(int argc, char **argv) {
    sockaddr_un mc_addr = create_udp_addr("/tmp/motion_controller_endpoint");
    sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, mc_addr);

    short x_amp = 0;
    short y_amp = 0;
    int forward_speed = 3;
    int turn_speed = 1;
    int max_amp = (turn_speed + forward_speed) * JoystickEvent::MAX_AXES_VALUE;

    while (true) {
        JoystickEvent event;
        socklen_t len = sizeof(js_addr);
        ssize_t bytes_read = recvfrom(
            sock,
            &event,
            sizeof(event),
            0,
            (sockaddr*)&js_addr,
            &len);
        if (bytes_read < 0) {
            perror("recvfrom");
            return 1;
        } else if (bytes_read != sizeof(event)) {
            usleep(1000);
            continue;
        }

        cout << "Packet received from time: " << event.time << endl;

        if (event.number == 0) { // Left X analog
            x_amp = event.value;
        } else if (event.number == 1) { // Right Y analog
            y_amp = event.value;
        }

        // We assume both left and right motors use positive magnitude to roll
        // "forward" w.r.t. the robot.
        float left_motor = clamp((forward_speed * y_amp + x_amp) / max_amp);
        float right_motor = clamp((forward_speed * y_amp - x_amp) / max_amp);

        // Send over serial port.
    }

    return 0;
}
