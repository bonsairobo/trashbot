#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

using namespace std;

static const size_t MAX_MSG_SIZE = 100;

static int setup_unix_socket(const char *socket_path) {
    struct sockaddr_un addr;
    int fd;

    if ( (fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
        perror("socket error");
        exit(-1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (*socket_path == '\0') {
        *addr.sun_path = '\0';
        strncpy(addr.sun_path+1, socket_path+1, sizeof(addr.sun_path)-2);
    } else {
        strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);
        unlink(socket_path);
    }

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("bind error");
        exit(-1);
    }

    if (listen(fd, 5) == -1) {
        perror("listen error");
        exit(-1);
    }

    return fd;
}

int main(int argc, char **argv) {
    int fd = setup_unix_socket("./joystick_data");
    while (true) {
        // Try to accept Joystick client connection.
        int cl;
        if ( (cl = accept(fd, NULL, NULL)) != -1) {
            int rc;
            char buf[100];
            while ( (rc = read(cl, buf, sizeof(buf))) > 0) {
                printf("read %u bytes: %.*s\n", rc, rc, buf);
            }

            if (rc == -1) {
                perror("read");
                exit(-1);
            }
            else if (rc == 0) {
                printf("EOF\n");
                close(cl);
            }
        }
    }
}
