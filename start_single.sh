#!/bin/bash
ros2 run hpe_test slave_single ${NODE_NAME} /camera/${CAMERA_NAME}/color/image_raw $1 /camera/${CAMERA_NAME}/color/camera_info
