# If you have trouble finding the Kinect device, you might need to run as root :(

root_dir=$PWD
sudo add-apt-repository ppa:george-edison55/cmake-3.x
sudo apt-get update
sudo apt-get install build-essential \
					 cmake \
					 libavcodec-dev \
					 libavformat-dev \
					 libswscale-dev \
					 python-dev \
					 python-numpy \
					 openjdk-6-jdk \
					 libgtk2.0-dev \
					 libudev-dev \
					 libusb-1.0-0-dev \
					 freeglut3-dev \
					 libxmu-dev \
					 libxi-dev

git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake -DBUILD_OPENNI2_DRIVER=ON -DCMAKE_BUILD_TYPE=RELEASE ..
cp lib/OpenNI2-FreenectDriver/libFreenectDriver.so ../../OpenNI2/Bin/x64-Release/OpenNI2/Drivers/
make
cd $root_dir
git clone https://github.com/occipital/OpenNI2.git
cd OpenNI2
make
cd $root_dir
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make -j4
sudo make install
cd $root_dir
mkdir joystickpp
cd joystickpp
git clone https://github.com/drewnoakes/joystick.git .
cd $root_dir
