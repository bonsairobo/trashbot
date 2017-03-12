root_dir=$PWD
sudo add-apt-repository ppa:george-edison55/cmake-3.x
sudo apt-get update
sudo apt-get install build-essential cmake openjdk-6-jdk libudev-dev libusb-1.0-0-dev freeglut3-dev libxmu-dev libxi-dev
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake .. -DBUILD_OPENNI2_DRIVER=ON
make
cd $root_dir
git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2
make
cd $root_dir
mkdir obj
