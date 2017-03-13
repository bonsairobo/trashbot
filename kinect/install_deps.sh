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
cmake .. -DBUILD_OPENNI2_DRIVER=ON
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
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j4
sudo make install
cd $root_dir
mkdir obj
