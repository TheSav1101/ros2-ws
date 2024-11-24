#!/bin/bash
TFLITE_DIR=~/tensorflow
BUILD_DIR=${TFLITE_DIR}/bazel-bin
VERSION=2.18.0
INSTALL_PREFIX=/usr/local
LIB_DIR=${INSTALL_PREFIX}/lib
INCLUDE_DIR=${INSTALL_PREFIX}/include/tensorflowlite
CMAKE_DIR=${INSTALL_PREFIX}/lib/cmake/tensorflowlite

if [ ! -f "${BUILD_DIR}/tensorflow/lite/libtensorflowlite.so" ]; then
    echo "Error: libtensorflowlite.so not found in ${BUILD_DIR}/tensorflow/lite/"
    exit 1
fi

sudo mkdir -p ${LIB_DIR} ${INCLUDE_DIR} ${CMAKE_DIR}

echo "Installing libtensorflowlite.so to ${LIB_DIR}"
sudo cp ${BUILD_DIR}/tensorflow/lite/libtensorflowlite.so ${LIB_DIR}

echo "Installing headers to ${INCLUDE_DIR}"
sudo cp -r ${TFLITE_DIR}/tensorflow/lite ${INCLUDE_DIR}

echo "Generating CMake configuration file"
cat <<EOF | sudo tee ${CMAKE_DIR}/tensorflowliteConfig.cmake > /dev/null
# CMake configuration file for TensorFlow Lite
set(TFLITE_INCLUDE_DIRS "${INCLUDE_DIR}/lite")
set(TFLITE_LIBRARY "${LIB_DIR}/libtensorflowlite.so")

# Provide variables for find_package
set(tensorflowlite_INCLUDE_DIRS \${TFLITE_INCLUDE_DIRS})
set(tensorflowlite_LIBRARIES \${TFLITE_LIBRARY})

# Targets
add_library(tensorflowlite SHARED IMPORTED)
set_target_properties(tensorflowlite PROPERTIES
    IMPORTED_LOCATION \${TFLITE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES \${TFLITE_INCLUDE_DIRS}
)
EOF

echo "Updating library cache"
sudo ldconfig

echo "TensorFlow Lite v${VERSION} installed successfully!"