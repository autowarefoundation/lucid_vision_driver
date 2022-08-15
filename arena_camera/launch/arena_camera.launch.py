from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python import get_package_share_directory
import os

# package path
arena_camera_node_prefix = get_package_share_directory('arena_camera')
arena_camera_node_prefix_param_file = os.path.join(arena_camera_node_prefix,
                                                   'param/lucid_vision_camera.param.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arena_camera',
            executable='arena_camera_node_exe',
            parameters=[arena_camera_node_prefix_param_file],
            # to see c++ codes
            output='screen'

        )
    ])
