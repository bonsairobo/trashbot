#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "joystick.hpp"

using namespace std;

int main(int argc, char **argv) {
    Joystick js;
    if (!js.isFound()) {
        cerr << "ERROR: No joystick found." << endl;
        return 1;
    }

    while (true) {
        usleep(1000);
        JoystickEvent event;
        if (js.sample(&event)) {
            if (event.isButton()) {
                cout << "Button " << (unsigned)event.number << " is "
                     << (event.value == 0 ? "up" : "down") << endl;
            } else if (event.isAxis()) {
                // cout << "Axis " << (unsigned)event.number
                //      << " is at position " << event.value << endl;
            }
        }
    }

    return 0;
}
