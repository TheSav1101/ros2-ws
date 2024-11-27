#!/bin/bash
ros2 run hpe_test slave_single ${NODE_NAME} /${CAMERA_NAME}/color/image_raw 0 /${CAMERA_NAME}/color/camera_info
