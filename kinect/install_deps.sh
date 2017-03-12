sudo apt-get install build-essential cmake libusb-1.0-0-dev freeglut3-dev libxmu-dev libxi-dev
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake ..
make
