# Copyright (c) 2020 OUXT Polaris
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robotx_twist_controller_parm_file = LaunchConfiguration(
        'robotx_twist_controller_parm_file',
        default=os.path.join(
            get_package_share_directory('robotx_twist_controller'),
            'config', 'robotx_twist_controller.yaml')
    )
    description = LaunchDescription([
        DeclareLaunchArgument(
            'robotx_twist_controller_parm_file',
            default_value=robotx_twist_controller_parm_file,
            description='robotx_twist_controller parameters'
        ),
        Node(
            package='robotx_twist_controller',
            node_executable='robotx_twist_controller_node',
            node_name='robotx_twist_controller',
            parameters=[robotx_twist_controller_parm_file],
            output='screen')
    ])
    return description
