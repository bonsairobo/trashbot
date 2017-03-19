#include "common.hpp"

using namespace std;
using namespace cv;
using namespace openni;

static int try_start_video_stream(
    VideoStream& stream,
    Device& device,
    SensorType type,
    const string& type_str,
    ostream& log_stream)
{
    Status rc;
    if (device.getSensorInfo(type) != NULL) {
        rc = stream.create(device, type);
        if (rc != STATUS_OK) {
            log_stream << "ERROR: Couldn't create " << type_str
                       << " stream" << endl
                       << OpenNI::getExtendedError() << endl;
            return 1;
        }
    }
    rc = stream.start();
    if (rc != STATUS_OK) {
        log_stream << "ERROR: Couldn't start the " << type_str
                   << " stream" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }
    return 0;
}

int try_start_rgbd_streams(
    Device& device,
    VideoStream& depth_stream,
    VideoStream& color_stream,
    ostream& log_stream)
{
    if (try_start_video_stream(
        color_stream, device, SENSOR_COLOR, "color", log_stream) != 0)
    {
        return 1;
    }
    if (try_start_video_stream(
        depth_stream, device, SENSOR_DEPTH, "depth", log_stream) != 0)
    {
        return 1;
    }

    // This ensures that depth and RGB streams are aligned with same resolution.
    if (device.isImageRegistrationModeSupported(
            IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    } else {
        cerr << "ERROR: image registration is not supported" << endl;
        return 1;
    }

    return 0;
}

// Convert OpenNI image format to OpenCV format.
static Mat cv_image_from_vframe_ref(const VideoFrameRef& frame, int n_bytes) {
    int type = n_bytes == 3 ? CV_8UC3 : CV_16UC1;
    Mat out(frame.getHeight(), frame.getWidth(), type);
    memcpy(out.data, frame.getData(),
        frame.getWidth() * frame.getHeight() * n_bytes);
    if (type == CV_8UC3)
        cvtColor(out, out, CV_RGB2BGR);
    return out;
}

int get_mat_from_stream(
    VideoStream& stream, Mat& mat, ostream& log_stream, int byte_depth)
{
    VideoFrameRef *frame = nullptr;
    Status rc = stream.readFrame(frame);
    if (rc != STATUS_OK or frame == nullptr) {
        log_stream << "Couldn't read depth frame." << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }
    mat = cv_image_from_vframe_ref(*frame, byte_depth);
    return 0;
}
