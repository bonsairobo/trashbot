#ifndef SOCKET_TYPES_HPP
#define SOCKET_TYPES_HPP

#include "../joystick/src/joystick.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

enum PacketType {
    NONE_TYPE,
    AXIS_EVENT,
    PICKUP_COMMAND,
    MODE_SWITCH_COMMAND
};

struct MCMotors {
    float l_motor, r_motor; // from -1.0 to 1.0
    MCMotors();
    MCMotors(float l, float r);
};

struct CodePacket {
    uint32_t time_ms;
    PacketType type;

    CodePacket(PacketType type);
};

// Just make a simple POD type so we don't have to use some library's type for
// serialization.
struct Vec3f {
    float x, y, z;
};

// Vectors are in Kinect coordinates.
struct GraspingPoint {
    uint32_t time_ms;
    Vec3f point;
    Vec3f normal;
    Vec3f principal_axis;
};

sockaddr_un create_udp_addr(const char *path);
int try_create_udp_socket();
void try_bind_path(int sock, sockaddr_un addr);

#endif // SOCKET_TYPES_HPP
