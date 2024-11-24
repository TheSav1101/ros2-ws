# Real-time multi-camera 3D human pose estimation on edge devices

In this workspace I leave the implementation of the code used in my thesis "Real-time multi-camera 3D human pose estimation on edge devices". Tests are made using a newtork of raspberry pi 5.

## Requirements:

    - ros2 jazzy
    - libtensorflow_lite, to be installed in "/usr/lib" with include files in "/usr/include/tensorflow"
    - cv_bridge
    - opencv4
    - eigen

## Usage


## Raspberry pi setup guide:

1. Install Ubuntu 24.10 using rpi-imager
2. Install opencv4 and eigen if needed
3. Install the latest version of ros2-jazzy and test using the premade examples
4. Install TFlite with GPU acceleration support:
'''
sudo apt update
sudo apt upgrade -y
sudo apt install cmake git build-essential clang
sudo apt install mesa-utils vulkan-tools clinfo
sudo apt-get install opencl-headers ocl-icd-opencl-dev libabsl-dev libruy-dev libpthreadpool-dev libxnnpack-dev
'''
'''
check if previous libs are working
'''
glxinfo | grep "OpenGL"
vulkaninfo | grep "Vulkan"
clinfo
'''
download and build tflite
'''
git clone https://github.com/tensorflow/tensorflow.git
cd tensorflow
git checkout v2.18.0 //or whatever new version you like
'''
Patch file ~/tensorflow/tensorflow/lite/tools/cmake/modules/ml_dtypes/CMakeLists.txt
'''
-target_include_directories(ml_dtypes INTERFACE
- "${ML_DTYPES_SOURCE_DIR}"
- "${ML_DTYPES_SOURCE_DIR}/ml_dtypes")
+target_include_directories(ml_dtypes INTERFACE
+  "$<BUILD_INTERFACE:${ML_DTYPES_SOURCE_DIR}>" "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
+  "$<BUILD_INTERFACE:${ML_DTYPES_SOURCE_DIR}/ml_dtypes>" "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ml_dtypes>")
'''

build

'''
cd
mkdir tflite_build
cd tflite_build

cmake -D TFLITE_ENABLE_GPU=ON \
  -D FETCHCONTENT_FULLY_DISCONNECTED=OFF \
  -D BUILD_SHARED_LIBS=ON \
  -D TFLITE_ENABLE_INSTALL=ON \
  -D CMAKE_FIND_PACKAGE_PREFER_CONFIG=ON \
  -D TFLITE_ENABLE_XNNPACK=ON \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D CMAKE_C_COMPILER=clang \
  -D CMAKE_CXX_COMPILER=clang++ \
  -D CMAKE_SYSTEM_PROCESSOR=aarch64 \
  ../tensorflow/tensorflow/lite

make -j $(nproc)
sudo make install
'''
