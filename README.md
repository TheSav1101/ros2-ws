# Real-time multi-camera 3D human pose estimation on edge devices
---

In this workspace I leave the implementation of the code used in my thesis "Real-time multi-camera 3D human pose estimation on edge devices". Tests are made using a newtork of raspberry pi 5.

# Usage
---

TODO

# Raspberry pi setup guide:
---
Install Ubuntu 24.04 server LTS using rpi-imager and ssh into the raspbery pi.

    sudo apt update && sudo apt upgrade -y
    sudo reboot

Clone this repository

    git clone https://github.com/TheSav1101/ros2-ws.git

Install the latest version of ros2-jazzy and test using the premade examples, you can follow this guide https://docs.ros.org/en/jazzy/Installation.html

    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt upgrade -y
    sudo apt install ros-jazzy-desktop-full -y
    sudo apt-get install colcon -y

Install opencv4 and other dependencies
    sudo apt update && sudo apt upgrade -y
    sudo apt install cmake git build-essential clang golang -y
    sudo apt install mesa-utils vulkan-tools clinfo -y
    sudo apt install libegl-dev libegl1-mesa-dev libx11-dev ocl-icd-dev -y
    sudo apt-get install opencl-headers ocl-icd-opencl-dev libabsl-dev libruy-dev libpthreadpool-dev libxnnpack-dev -y
    git clone https://github.com/google/flatbuffers.git -b v24.3.25
    cd flatbuffers
    mkdir build
    cd build
    cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install

Add these lines to ~/.bashrc and restart bash aftewards

    source /opt/ros/jazzy/setup.bash
    source ~/ros2-ws/install/setup.bash
    export PATH=$PATH:$(go env GOPATH)/bin
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

Install bazel

    go install github.com/bazelbuild/bazelisk@latest

Download and build tflite 2.17.0

    git clone https://github.com/tensorflow/tensorflow.git
    cd tensorflow
    git checkout v2.17.0

Run configuration and set all defaults. Note that you have to invoke bazelisk before it in order to download the package or it won't work

    bazelsik
    ./configure

Run builds with bazelisk

    bazelisk build -c opt --config=monolithic //tensorflow/lite:libtensorflowlite.so
    bazelisk build -c opt --config=monolithic --linkopt=-lEGL --linkopt=-lGLESv2 //tensorflow/lite/delegates/gpu:libtensorflowlite_gpu_delegate.so


Install libtensorflowlite.so and the gpu delegate with the provided script

    cd ~/ros2-ws
    ./install_tflite.sh

And in the end you can:

    colcon build --packages-select hpe_msgs
    colcon build
