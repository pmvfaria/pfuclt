#!/bin/bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update
sudo apt-get install build-essential software-properties-common -y
sudo apt-get install gcc-snapshot gcc-8 g++-8 -y

printf "\n----\n"

echo "DONE"
echo "Now you must link to the new version before compilation"
echo "1. For system-wide:"
echo "    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-8"
echo "2. For this workspace only:"
echo "    catkin clean"
echo "    catkin config --cmake-args -DCMAKE_C_COMPILER=/usr/bin/gcc-8 -DCMAKE_CXX_COMPILER=/usr/bin/g++-8" 
