#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <cmath>
#include <queue>
#include <cassert>

using namespace std;

static int create_and_listen_unix_socket(const char *socket_path) {
    struct sockaddr_un addr;
    int sock;

    if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        cerr << "ERROR: could not create socket" << endl;
        exit(-1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;

    // Copy one byte fewer to ensure there is a null-terminator.
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);

    // If the path already exists, must unlink before rebinding.
    if (access(addr.sun_path, F_OK) == 0)
        unlink(addr.sun_path);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        cerr << "ERROR: could not bind socket" << endl;
        exit(-1);
    }

    if (listen(sock, 5) == -1) {
        cerr << "ERROR: could not listen on socket" << endl;
        exit(-1);
    }

    return sock;
}

struct JoystickData {

};

template<typename T>
class StreamSequencer {
    queue<T> data_queue;
    size_t cursor;
    T new_data;

public:
    StreamSequencer(): cursor(0) {}

    void handle_buffer(char *buf, size_t bytes_read) {
        assert(bytes_read <= sizeof(T));
        size_t n_copy = min(bytes_read, sizeof(T) - cursor);
        memcpy((char*)&new_data + cursor, buf, n_copy);
        cursor = cursor + n_copy;
        if (cursor == sizeof(T)) {
            data_queue.push(new_data);
            size_t rem = bytes_read - n_copy;
            memcpy((char*)&new_data, buf + n_copy, rem);
            cursor = rem;
        }
    }
};

int main(int argc, char **argv) {
    int sock = create_and_listen_unix_socket("./joystick_data");
    while (true) {
        // Try to accept Joystick client connection.
        int client;
        if ((client = accept(sock, NULL, NULL)) != -1) {
            StreamSequencer<JoystickData> seq;
            size_t bytes_read;
            do {
                // Joystick packets are small, so buffer can be small.
                char buf[64];
                bytes_read = read(client, buf, sizeof(buf));
                if (bytes_read < 0) {
                    cerr << "ERROR: could not read from socket" << endl;
                    return 1;
                }
                seq.handle_buffer(buf, bytes_read);
            } while (bytes_read > 0);
            close(client);
        }
    }

    return 0;
}
