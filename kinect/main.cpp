#include <libfreenect/libfreenect.h>
#include <iostream>
#include <signal.h>

using namespace std;

void depth_cb(freenect_device* dev, void* data, uint32_t timestamp) {
    cout << "Received depth frame at " << timestamp << endl;
}

void video_cb(freenect_device* dev, void* data, uint32_t timestamp) {
    cout << "Received video frame at " << timestamp << endl;
}

volatile bool running = true;
void signalHandler(int signal)
{
    if (signal == SIGINT || signal == SIGTERM || signal == SIGQUIT)
        running = false;
}

int main(int argc, char** argv) {
    // Initialize libfreenect.
    freenect_context* fn_ctx;
    int ret = freenect_init(&fn_ctx, NULL);
    if (ret < 0)
        return ret;

    // Show debug messages.
    freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);

    // Find out how many devices are connected.
    int num_devices = ret = freenect_num_devices(fn_ctx);
    if (ret < 0)
        return ret;
    if (num_devices == 0) {
        cout << "No device found!" << endl;
        freenect_shutdown(fn_ctx);
        return 1;
    }

    // Open the first device.
    freenect_device* fn_dev;
    ret = freenect_open_device(fn_ctx, &fn_dev, 0);
    if (ret < 0) {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    // Set depth and video modes.
    ret = freenect_set_depth_mode(
        fn_dev,
        freenect_find_depth_mode(
            FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    if (ret < 0) {
        freenect_shutdown(fn_ctx);
        return ret;
    }
    ret = freenect_set_video_mode(
        fn_dev,
        freenect_find_video_mode(
            FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    if (ret < 0) {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    // Set frame callbacks.
    freenect_set_depth_callback(fn_dev, depth_cb);
    freenect_set_video_callback(fn_dev, video_cb);

    // Start depth and video.
    ret = freenect_start_depth(fn_dev);
    if (ret < 0) {
        freenect_shutdown(fn_ctx);
        return ret;
    }
    ret = freenect_start_video(fn_dev);
    if (ret < 0) {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    // Run until interruption or failure.
    while (running && freenect_process_events(fn_ctx) >= 0) {}

    cout << "Shutting down" << endl;

    // Stop everything and shutdown.
    freenect_stop_depth(fn_dev);
    freenect_stop_video(fn_dev);
    freenect_close_device(fn_dev);
    freenect_shutdown(fn_ctx);

    cout << "Done!" << endl;

    return 0;
}
