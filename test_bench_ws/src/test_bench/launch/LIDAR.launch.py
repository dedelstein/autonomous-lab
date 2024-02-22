import os

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    lidar_node = Node(
        package = "sick_scan_xd",
        executable = "sick_mrs_6xxx",
        parameters=[{
            'frame_id': 'LIDAR',
            'hostname': '192.168.0.2'
            }]
        )
    return LaunchDescription([
        lidar_node
    ])