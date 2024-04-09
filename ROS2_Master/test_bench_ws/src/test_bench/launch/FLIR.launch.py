import os

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

pkg_path = os.path.join(get_package_share_directory('test_bench'))
FLIR_params = os.path.join(pkg_path,'config','FLIR_camera.yaml')

example_params = {
            'debug': False,
            'compute_brightness': False,
            'dump_node_map': False,
            # set parameters defined in chameleon.yaml
            'gain_auto': 'Continuous',
            'exposure_auto': 'Continuous',
            'offset_x': 0,
            'offset_y': 0,
            'image_width': 2048,
            'image_height': 1536,
            'pixel_format': 'RGB8',  # 'BayerRG8, 'RGB8' or 'Mono8'
            'frame_rate_continous': True,
            'frame_rate': 100.0,
            'trigger_mode': 'Off',
            'chunk_mode_active': True,
            'chunk_selector_frame_id': 'FrameID',
            'chunk_enable_frame_id': True,
            'chunk_selector_exposure_time': 'ExposureTime',
            'chunk_enable_exposure_time': True,
            'chunk_selector_gain': 'Gain',
            'chunk_enable_gain': True,
            'chunk_selector_timestamp': 'Timestamp',
            'chunk_enable_timestamp': True
            }

def generate_launch_description():

    flir_node = Node(
        package = "spinnaker_camera_driver",
        executable = "camera_driver_node",
        output='screen',
        namespace='camera',
        parameters=[
            example_params,
            {
            'parameter_file': FLIR_params,
            'camera_type': 'chameleon',
            'serial_number': '20073275',
            'frame_id': 'FLIR'
            }]
        )
    return LaunchDescription([
        flir_node
    ])