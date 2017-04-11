#!/bin/bash
#Install467
sudo apt-get update
sudo apt-get -y install autoconf automake autotools-dev libglib2.0-dev manpages-dev manpages-posix-dev libgl1-mesa-dev gtk-doc-tools libgtk2.0-dev python-dev libusb-dev libusb-1.0-0-dev libdc1394-22-dev libdc1394-utils libfuse-dev libi2c-dev libgsl0-dev gsl-doc-info gsl-doc-pdf freeglut3-dev
sudo apt-get -y install exfat-fuse exfat-utils
# camera & openCV stuff
sudo apt-get -y install coriander libjpeg8-dev libtiff-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libatlas-base-dev gfortran
# java stuff
sudo apt-get -y install ant openjdk-8-jdk
# additional stuff
sudo apt-get -y install cmake
# python stuff
sudo apt-get -y install python-dev python-cairo python-pygame python-matplotlib python-numpy python-scipy python-pyaudio python-tk ipython pyqt4-dev-tools

#Install LCM
wget https://github.com/lcm-proj/lcm/releases/download/v1.2.1/lcm-1.2.1.zip
unzip lcm-1.2.1.zip
rm lcm-1.2.1.zip
cd lcm-1.2.1/
./configure
make
sudo make install
