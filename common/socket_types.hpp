#ifndef SOCKET_TYPES_HPP
#define SOCKET_TYPES_HPP

#include "../joystick/src/joystick.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

struct PickupCommand {
    uint32_t time_ms;
};

// Points are in Camera coordinates. The outer vector is sorted, with highest
// probability grasping points at the front. The inner vector contains a cluster
// of points for a single grasp.
struct GraspingPoints {
    uint32_t time_ms;
    std::vector<std::vector<cv::Point3f>> points;
};

sockaddr_un create_udp_addr(const char *path);

int try_create_udp_socket();

void try_bind_path(int sock, sockaddr_un addr);

#endif // SOCKET_TYPES_HPP
