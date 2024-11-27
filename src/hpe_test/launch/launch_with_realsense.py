import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'rgb_camera.color_profile:=640x480x30',
                'pointcloud.enable:=false',
                'depth_module.enable:=false',
                'camera_name:=${CAMERA_NAME}'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'hpe_test', 'slave_single',
                '${NODE_NAME}', '/${CAMERA_NAME}/color/image_raw',
                '/${CAMERA_NAME}/color/camera_info'
            ],
            output='screen'
        ),
    ])
