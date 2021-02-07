#!/bin/bash

# Installs ROS, PROS and gcc-arm compiler for V5 builds

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt upgrade -y
sudo apt install python3 python-is-python3 python3-pip ros-noetic-desktop-full python3-rosdep git -y
sudo rosdep init
rosdep update

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source ~/git/ChangeUp/devel/setup.bash' >> ~/.bashrc

# Setup X forwarding
echo "export DISPLAY=localhost:0.0" >> ~/.bashrc
sudo strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5  # Fix for WSL 1 bug

# Install gcc-arm compiler
cd ~
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 --no-check-certificate
tar -xjvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2
echo 'export PATH=$PATH:~/gcc-arm-none-eabi-9-2020-q2-update/bin/' >> ~/.bashrc
source ~/.bashrc
arm-none-eabi-gcc --version

# Install PROS
sudo python3.8 -m pip install https://github.com/purduesigbots/pros-cli/releases/download/3.1.4/pros_cli_v5-3.1.4-py3-none-any.whl

# Clone ChangeUp repository
cd ~
mkdir git
cd git
git clone git@github.com:msoe-vex/ChangeUp.git
cd ChangeUp
git submodule init
git submodule update

# Build repository
catkin_make