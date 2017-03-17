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

    // This process is the sender.
    sockaddr_un mc_recv_addr =
        create_udp_addr("/tmp/motion_controller_endpoint");
    sockaddr_un kin_recv_addr = create_udp_addr("/tmp/kinect_endpoint");
    sockaddr_un send_addr = create_udp_addr("/tmp/joystick_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, send_addr);

    while (true) {
        usleep(1000);
        JoystickEvent event;
        while (js.sample(&event)) {
            // Press any button to send a pickup command.
            if (event.isButton() and event.value == 1) {
                PickupCommand cmd;
                cmd.time_ms = event.time;
                sendto(
                    sock,
                    &cmd,
                    sizeof(cmd),
                    0,
                    (sockaddr*)&kin_recv_addr,
                    sizeof(kin_recv_addr));
            } else if (event.isAxis() and
                // TODO: get actual axis #s
                (event.number == 0 or event.number == 1))
            {
                sendto(
                    sock,
                    &event,
                    sizeof(event),
                    0,
                    (sockaddr*)&mc_recv_addr,
                    sizeof(mc_recv_addr));
            }
        }
    }

    return 0;
}
