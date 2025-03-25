#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
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


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    champ_base_package = FindPackageShare('champ_base')
    base_to_footprint_ekf_config_path = PathJoinSubstitution(
        [champ_base_package, 'config', 'ekf', 'base_to_footprint.yaml']
    )
    footprint_to_odom_ekf_config_path = PathJoinSubstitution(
        [champ_base_package, 'config', 'ekf', 'footprint_to_odom.yaml']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    base_to_footprint_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='base_to_footprint_ekf',
        output='screen',
        parameters=[
            {'base_link_frame': 'base_link'},
            {'use_sim_time': use_sim_time},
            base_to_footprint_ekf_config_path,
        ],
        remappings=[('odometry/filtered', 'odom/local')]
    )

    footprint_to_odom_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='footprint_to_odom_ekf',
        output='screen',
        parameters=[
            {'base_link_frame': 'base_link'},
            {'use_sim_time': use_sim_time},
            footprint_to_odom_ekf_config_path,
        ],
        remappings=[('odometry/filtered', 'odom')]
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        base_to_footprint_ekf,
        footprint_to_odom_ekf
    ])
