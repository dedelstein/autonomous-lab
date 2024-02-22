import os

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    flir_node = Node(
        package = "spinnaker_camera_driver",
        executable = "camera_driver_node",
        output='screen',
        namespace='camera',
        parameters={
            'camera_type': 'chameleon',
            'serial_number': '20073275',
            'frame_id': 'FLIR'
            }
        )
    return LaunchDescription([
        flir_node
    ])