# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchContext

import yaml


def generate_launch_description():
    launch_arguments = []

    context = LaunchContext()
    camera_param_path = os.path.join(
        FindPackageShare("lucid_vision_driver").perform(context),
        "param/test.param.yaml"
    )
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    container = ComposableNodeContainer(
        name="camera_node_right",
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="lucid_vision_driver",
                plugin="ArenaCameraNode",
                name="arena_camera_node_right",
                parameters=[{"camera_name": camera_yaml_param['camera_name'],
                             "frame_id": camera_yaml_param['frame_id'],
                             "pixel_format": camera_yaml_param['pixel_format'],
                             "serial_no": camera_yaml_param['serial_no'],
                             "camera_info_url": camera_yaml_param['camera_info_url'],
                             "fps": camera_yaml_param['fps'],
                             "horizontal_binning": camera_yaml_param['horizontal_binning'],
                             "vertical_binning": camera_yaml_param['vertical_binning'],
                             "use_default_device_settings": camera_yaml_param['use_default_device_settings'],
                             "exposure_auto": camera_yaml_param['exposure_auto'],
                             "exposure_target": camera_yaml_param['exposure_target'],
                             "gain_auto": camera_yaml_param['gain_auto'],
                             "gain_target": camera_yaml_param['gain_target'],
                             "gamma_target": camera_yaml_param['gamma_target'],
                             "enable_compressing": camera_yaml_param['enable_compressing'],
                             "enable_rectifying": camera_yaml_param['enable_rectifying'],
                             }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="both",
    )

    return LaunchDescription(
        [
            *launch_arguments,
            container,
        ]
    )