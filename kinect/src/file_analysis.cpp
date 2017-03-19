#include "img_proc.hpp"
#include "common.hpp"

using namespace std;
using namespace cv;
using namespace openni;

int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "Need the ONI file location!" << endl;
        return 1;
    }

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        cerr << "Initialize failed" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    Device device;
    rc = device.open(argv[1]);
    if (rc != STATUS_OK) {
        cerr << "Couldn't open device" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    VideoStream depth_stream, color_stream;
    if (try_start_rgbd_streams(
        device, depth_stream, color_stream, cout) != 0)
    {
        return 1;
    }

    namedWindow("debug", 1);

    // Seek through recording by depth frame index.
    PlaybackControl *pbc = device.getPlaybackControl();
    pbc->setSpeed(-1); // Make sure reading never blocks.
    int num_depth_frames = pbc->getNumberOfFrames(depth_stream);
    for (int i = 0; i < num_depth_frames; ++i) {
        // This should set also set color stream to the same point in time.
        pbc->seek(depth_stream, i);

        Mat depth_mat, color_mat;
        if (get_mat_from_stream(depth_stream, depth_mat, cout, 2, nullptr) != 0)
        {
            return 1;
        }
        if (get_mat_from_stream(color_stream, color_mat, cout, 3, nullptr) != 0)
        {
            return 1;
        }

        // Do image analysis.
    }

    OpenNI::shutdown();
    return 0;
}
