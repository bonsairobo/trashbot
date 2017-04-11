#!/bin/bash
#Install lcm. Need this to communicate with rexarm
wget https://github.com/lcm-proj/lcm/releases/download/v1.2.1/lcm-1.2.1.zip
unzip lcm-1.2.1.zip
rm lcm-1.2.1.zip
cd lcm-1.2.1/
./configure
make
sudo make install
