# Copyright (c) 2026 Franka Robotics GmbH
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_parameters_file = os.path.join(
        get_package_share_directory('franka_gripper_teleop'),
        'config',
        'gripper_teleop.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'parameters_file',
                default_value=default_parameters_file,
                description='Path to the YAML file containing node parameters.',
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace of the teleoperation node.',
            ),
            Node(
                package='franka_gripper_teleop',
                executable='gripper_teleop_node',
                name='gripper_teleop_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[LaunchConfiguration('parameters_file')],
                output='screen',
            ),
        ]
    )
