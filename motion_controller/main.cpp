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
    ofstream arduino("/dev/ttyACM1");
    if (!arduino.is_open()) {
        cerr << "No arduino present." << endl;
        return 1;
    }

    const int forward_speed = 1;
    const int turn_speed = 2;
    const int max_amp =
        (turn_speed + forward_speed) * JoystickEvent::MAX_AXES_VALUE;
    bool manual_mode = true;
    int x_amp = 0;
    int y_amp = 0;
    uint8_t prev_left_motor_byte = 0, prev_right_motor_byte = 0;

    float left_motor = 0.0, right_motor = 0.0;
    while (true) {
        // The motion controller can be in 2 modes: manual control or
        // autonomous control. In manual control, the motion controller receives
        // direct input from the joystick. In autonomous control mode, the
        // motion controller receives input from the Kinect process.
        if (manual_mode) {
            CodePacket code(NONE_TYPE);
            JoystickEvent event;
            sockaddr_un addr;
            ssize_t bytes_read = recvfrom(
                sock,
                &code,
                sizeof(code),
                0,
                (sockaddr*)&addr,
                &len);
            if (bytes_read < 0) {
                if (errno == EAGAIN) {
                    usleep(10);
                    continue;
                }
                perror("recvfrom");
                return 1;
            }
            if (strcmp(addr.sun_path, js_addr.sun_path) == 0) {
                if (code.type == AXIS_EVENT) {
                    // Wait for packet... forever!
                    while (true) {
                        bytes_read = recvfrom(
                            sock,
                            &event,
                            sizeof(event),
                            0,
                            (sockaddr*)&addr,
                            &len);
                        if (bytes_read < 0) {
                            if (errno == EAGAIN) {
                                usleep(10);
                                continue;
                            }
                            perror("recvfrom");
                            return 1;
                        }
                        if (strcmp(addr.sun_path, js_addr.sun_path) == 0) {
                            break;
                        }
                    }
                } else if (code.type == MODE_SWITCH_COMMAND) {
                    cout << "ENTERING AUTONOMOUS MODE" << endl;
                    manual_mode = false;
                }
            }

            if (event.number == 2) { // Left X analog
                x_amp = event.value;
            } else if (event.number == 12) { // Left trigger
                y_amp = -event.value;
            } else if (event.number == 13) { // Right trigger
                y_amp = event.value;
            }

            cout << "Packet received from time: " << event.time << endl;

            // We assume both left and right motors use positive magnitude to
            // roll "forward" w.r.t. the robot.
            left_motor = clamp(
                float(forward_speed * y_amp + x_amp) / max_amp);
            right_motor = clamp(
                float(forward_speed * y_amp - x_amp) / max_amp);
        } else /* AUTONOMOUS MODE */ {
            // Autonomous driving. Kinect senses obstacles and objects, while
            // telling the motion controller how to steer.
            // Block until we get new motor values, since the Arduino will
            // continue to apply the previous motor values.
            ssize_t bytes_read = 0;
            while (bytes_read <= 0) {
                // Here, we may receive from two different addresses, the kinect
                // or the joystick. Thus, we must check the sender address
                // before copying the buffer into a specific type.

                // Both packet types are 8 bytes large.
                uint8_t buffer[8];
                sockaddr_un addr;
                bytes_read = recvfrom(
                    sock,
                    &buffer,
                    sizeof(buffer),
                    0,
                    (sockaddr*)&addr,
                    &len);
                if (bytes_read < 0) {
                    if (errno != EAGAIN) {
                        perror("recvfrom");
                        return 1;
                    }
                }
                if (strcmp(addr.sun_path, js_addr.sun_path) == 0) {
                    if (bytes_read == sizeof(CodePacket)) {
                        CodePacket code(NONE_TYPE);
                        memcpy(&code, buffer, sizeof(code));
                        if (code.type == MODE_SWITCH_COMMAND) {
                            cout << "ENTERING MANUAL MODE" << endl;
                            left_motor = 0.0;
                            right_motor = 0.0;
                            manual_mode = true;
                            break;
                        }
                    }
                } else if (strcmp(addr.sun_path, kin_addr.sun_path) == 0) {
                    if (bytes_read == sizeof(MCMotors)) {
                        MCMotors motors;
                        memcpy(&motors, buffer, sizeof(motors));
                        left_motor = motors.l_motor;
                        right_motor = motors.r_motor;
                    }
                }
            }
        }

        uint8_t left_motor_byte = round(abs(left_motor) * 255);
        uint8_t right_motor_byte = round(abs(right_motor) * 255);
        char left_sgn = left_motor >= 0 ? '+' : '-';
        char right_sgn = right_motor >= 0 ? '+' : '-';

        cout << "motor bytes = " << int(left_motor_byte) << " "
             << int(right_motor_byte) << endl;

        // Don't send duplicate motor bytes,
        if (left_motor_byte != prev_left_motor_byte or
            right_motor_byte != prev_right_motor_byte)
        {
            // Send over serial port.
            arduino << 'L' << left_sgn << left_motor_byte
                    << 'R' << right_sgn << right_motor_byte;
            arduino.flush();
        }

        prev_left_motor_byte = left_motor_byte;
        prev_right_motor_byte = right_motor_byte;
    }

    arduino.close();

    return 0;
}
