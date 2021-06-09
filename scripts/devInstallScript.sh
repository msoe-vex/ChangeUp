#!/bin/bash

# Installs ROS, PROS and gcc-arm compiler for V5 builds

# Install ROS
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install curl -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install python3 python-is-python3 python3-pip ros-noetic-desktop-full -y
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc

# Install gcc-arm compiler
mkdir ~/downloads && sudo mkdir /gcc && cd ~/downloads
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 --no-check-certificate
sudo tar -xjvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 -C /gcc
echo 'export PATH=$PATH:/gcc/gcc-arm-none-eabi-9-2020-q2-update/bin/' >> ~/.bashrc
source ~/.bashrc
arm-none-eabi-gcc --version

# Install PROS
sudo python3.8 -m pip install https://github.com/purduesigbots/pros-cli/releases/download/3.1.4/pros_cli_v5-3.1.4-py3-none-any.whl
