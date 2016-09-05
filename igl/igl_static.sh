#!/bin/sh

sudo mkdir /usr/include/libiglbin
sudo chown -R $USER /usr/include/libiglbin/
cp CMakeLists.txt /usr/include/libiglbin/
cd /usr/include/libiglbin
cmake
make

