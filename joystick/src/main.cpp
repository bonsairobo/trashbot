#include "../../common/socket_types.hpp"
#include "joystick.hpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    Joystick js;
    if (!js.isFound()) {
        cerr << "ERROR: No joystick found." << endl;
        return 1;
    }

    sockaddr_un mc_addr =
        create_udp_addr("/tmp/motion_controller_endpoint");
    sockaddr_un kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, js_addr);

    while (true) {
        usleep(1000);
        JoystickEvent event;
        while (js.sample(&event)) {
            // Press button to send a pickup command or switch between manual
            // and Roomba modes.
            if (event.isButton() and event.value == 1) {
                if (event.number == 12 or event.number == 14) {
                    cout << int(event.number) << ": " << event.value << endl;
                    CodePacket cmd(event.number);
                    cmd.time_ms = event.time;
                    sendto(
                        sock,
                        &cmd,
                        sizeof(cmd),
                        0,
                        (sockaddr*)&kin_addr,
                        sizeof(kin_addr));
                }
            } else if (event.isAxis() and
                // TODO: get actual axis #s for left X analog and right Y analog
                (event.number == 2 or event.number == 12 or event.number == 13))
            {
                cout << int(event.number) << ": " << event.value << endl;
                sendto(
                    sock,
                    &event,
                    sizeof(event),
                    0,
                    (sockaddr*)&mc_addr,
                    sizeof(mc_addr));
            }
        }
    }

    return 0;
}
