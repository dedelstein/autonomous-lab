import os

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    flir_node = Node(
        package = "spinnaker_camera_driver",
        executable = "driver_node",
        output='screen',
        namespace='camera',
        parameters=[{
            'camera_type': 'chameleon',
            'serial': '20073275',
            'frame_id': 'FLIR'
            }]
        )
    return LaunchDescription([
        flir_node
    ])