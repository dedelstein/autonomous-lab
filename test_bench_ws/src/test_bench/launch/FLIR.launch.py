import os

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

pkg_path = os.path.join(get_package_share_directory('test_bench'))
FLIR_params = os.path.join(pkg_path,'config','FLIR_camera.yaml')

def generate_launch_description():

    flir_node = Node(
        package = "spinnaker_camera_driver",
        executable = "camera_driver_node",
        output='screen',
        namespace='camera',
        parameters=[{
            'parameter_file': FLIR_params,
            'camera_type': 'chameleon',
            'serial_number': '20073275',
            'frame_id': 'FLIR'
            }]
        )
    return LaunchDescription([
        flir_node
    ])