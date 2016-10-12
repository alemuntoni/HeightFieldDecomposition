#!/bin/sh

sudo apt-get install cmake libcgal-qt5-dev -y
sudo apt-get install libblas-dev liblapack-dev -y
sudo apt-get install xorg-dev libglu1-mesa-dev -y

sudo mkdir /usr/include/libiglbin
sudo chown -R $USER /usr/include/libiglbin/
cp CMakeLists.txt /usr/include/libiglbin/
cd /usr/include/libiglbin
cmake .
make

