# If you have trouble finding the Kinect device, you might need to run as root :(

root_dir=$PWD
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
