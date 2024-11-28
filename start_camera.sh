#!/bin/bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x30 \
  pointcloud.enable:=false \
  enable_depth:=false \
  camera_name:=${CAMERA_NAME}
