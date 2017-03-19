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

    NIManager ni_man;
    ni_man.open(false, cout);

    namedWindow("debug", 1);

    // Seek through recording by depth frame index.
    PlaybackControl *pbc = ni_man.device.getPlaybackControl();
    int num_depth_frames = pbc->getNumberOfFrames(ni_man.depth_stream);
    for (int i = 0; i < num_depth_frames; ++i) {
        // This should set also set color stream to the same point in time.
        pbc->seek(ni_man.depth_stream, i);

        Mat depth_mat, color_mat;
        if (get_mat_from_stream(ni_man.depth_stream, depth_mat, cout, 2) != 0) {
            return 1;
        }
        if (get_mat_from_stream(ni_man.color_stream, color_mat, cout, 3) != 0) {
            return 1;
        }

        // Do image analysis.
    }

    return 0;
}
