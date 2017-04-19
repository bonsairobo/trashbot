#include <iostream>
#include <cmath>
#include <queue>
#include "../common/socket_types.hpp"
#include <cassert>
#include <fstream>
#include <fcntl.h>

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
    sockaddr_un kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    socklen_t len = sizeof(js_addr);
    int sock = try_create_udp_socket();
    try_bind_path(sock, mc_addr);
    fcntl(sock, F_SETFL, O_NONBLOCK);

    // Open serial port to Arduino.
    ofstream arduino("/dev/ttyACM0");
    if (!arduino.is_open()) {
        cerr << "No arduino present" << endl;
        return 1;
    }

    const int forward_speed = 1;
    const int turn_speed = 2;
    const int max_amp =
        (turn_speed + forward_speed) * JoystickEvent::MAX_AXES_VALUE;
    bool manual_mode = true;
    int x_amp = 0;
    int y_amp = 0;

    float left_motor, right_motor;
    while (true) {
        // The motion controller can be in 2 modes: manual control or
        // autonomous control. In manual control, the motion controller receives
        // direct input from the joystick. In autonomous control mode, the
        // motion controller receives input from the Kinect process.
        if (manual_mode) {
            CodePacket code(NONE_TYPE);
            JoystickEvent event;
            ssize_t bytes_read = recvfrom(
                sock,
                &code,
                sizeof(code),
                0,
                (sockaddr*)&js_addr,
                &len);
            if (bytes_read < 0) {
                perror("recvfrom");
                return 1;
            }
            if (bytes_read == EWOULDBLOCK) {
                continue;
            }
            if (code.type == AXIS_EVENT) {
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
            } else if (code.type == MODE_SWITCH_COMMAND) {
                manual_mode = !manual_mode;
            }

            if (event.number == 2) { // Left X analog
                x_amp = event.value;
            } else if (event.number == 12) { // Left trigger
                y_amp = -event.value;
            } else if (event.number == 13) { // Right trigger
                y_amp = event.value;
            }

            cout << "Packet received from time: " << event.time << endl;

            // We assume both left and right motors use positive magnitude to roll
            // "forward" w.r.t. the robot.
            left_motor =
                clamp(float(forward_speed * y_amp + x_amp) / max_amp);
            right_motor =
                clamp(float(forward_speed * y_amp - x_amp) / max_amp);
        } else /* AUTONOMOUS MODE */ {
            // Check if the joystick wants to turn off autonomous mode.
            CodePacket code(NONE_TYPE);
            ssize_t bytes_read = recvfrom(
                sock,
                &code,
                sizeof(code),
                0,
                (sockaddr*)&js_addr,
                &len);
            if (bytes_read < 0) {
                perror("recvfrom");
                return 1;
            }
            if (code.type == MODE_SWITCH_COMMAND) {
                manual_mode = !manual_mode;
            }

            // Autonomous driving. Kinect senses obstacles and objects, while
            // telling the motion controller how to steer.
            MCMotors motors;
            bytes_read = recvfrom(
                sock,
                &motors,
                sizeof(motors),
                0,
                (sockaddr*)&kin_addr,
                &len);
            left_motor = motors.l_motor;
            right_motor = motors.r_motor;
        }

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
