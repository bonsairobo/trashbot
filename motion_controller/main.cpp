#include <iostream>
#include <cmath>
#include <queue>
#include "../common/socket_types.hpp"
#include <cassert>
#include <fstream>

using namespace std;

float clamp(float x) {
    if (x < -1.0)
        return -1.0;
    if (x > 1.0)
        return 1.0;
    return x;
}

template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main(int argc, char **argv) {
    sockaddr_un mc_addr = create_udp_addr("/tmp/motion_controller_endpoint");
    sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, mc_addr);

    // Open serial port to Arduino.
    ofstream arduino("/dev/ttyACM0");
    if (!arduino.is_open()) {
        cerr << "No arduino present" << endl;
        return 1;
    }

    int x_amp = 0;
    int y_amp = 0;
    int forward_speed = 1;
    int turn_speed = 2;
    int max_amp = (turn_speed + forward_speed) * JoystickEvent::MAX_AXES_VALUE;

    while (true) {
        JoystickEvent event;
        socklen_t len = sizeof(js_addr);
        // Block until there is something to read.
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
        }

        cout << "Packet received from time: " << event.time << endl;

        if (event.number == 2) { // Left X analog
            x_amp = event.value;
        } else if (event.number == 12) { // Left trigger
            y_amp = -event.value;
        } else if (event.number == 13) { // Right trigger
            y_amp = event.value;
        }

        // We assume both left and right motors use positive magnitude to roll
        // "forward" w.r.t. the robot.
        float left_motor =
            clamp(float(forward_speed * y_amp + x_amp) / max_amp);
        float right_motor =
            clamp(float(forward_speed * y_amp - x_amp) / max_amp);
        unsigned char left_motor_byte = round(abs(left_motor) * 255);
        unsigned char right_motor_byte = round(abs(right_motor) * 255);
        char left_sgn = left_motor >= 0 ? '+' : '-';
        char right_sgn = right_motor >= 0 ? '+' : '-';

        cout << left_motor << " " << right_motor << endl;

        // Send over serial port.
        arduino << 'L' << left_sgn << left_motor_byte
                << 'R' << right_sgn << right_motor_byte;
        arduino.flush();
    }

    arduino.close();

    return 0;
}
