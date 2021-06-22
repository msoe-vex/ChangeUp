FROM ros:noetic-ros-base-focal

RUN apt-get update -y && \
    apt-get upgrade -y ros-noetic-robot=1.5.0-1*
    
# RUN apt-get update -y && apt-get upgrade -y && apt-get install -y \
#     ros-noetic-robot=1.5.0-1* && \
#     ros-noetic-perception=1.5.0-1* && \
#     rm -rf /var/lib/apt/lists/*

# RUN apt install -y \
#     python3 && \
#     python3-pip && \
#     python-is-python3 && \
#     wget

# RUN mkdir ~/downloads && \
#     cd ~/downloads && \
#     wget https://armkeil.blob.`core.windows.net/developer/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 --no-check-certificate && \
#     tar -xjvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz2 && \
#     echo 'export PATH=$PATH:~/gcc-arm-none-eabi-9-2020-q2-update/bin/' >> ~/.bashrc && \
#     source ~/.bashrc && \
#     arm-none-eabi-gcc --version

# RUN sudo python3.8 -m pip install https://github.com/purduesigbots/pros-cli/releases/download/3.1.4/pros_cli_v5-3.1.4-py3-none-any.whl