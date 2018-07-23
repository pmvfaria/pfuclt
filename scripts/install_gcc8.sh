#!/bin/bash
sudo apt-get update && \
    sudo apt-get install build-essential software-properties-common -y && \
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    sudo apt-get update && \
    sudo apt-get install gcc-snapshot -y && \
    sudo apt-get update && \
    sudo apt-get install gcc-8 g++-8 -y && \
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-8
