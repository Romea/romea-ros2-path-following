# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace, SetParameter


import romea_joystick_bringup
from romea_joystick_description import joystick_buttons_remapping

import romea_mobile_base_bringup
from romea_mobile_base_description import (
    get_mobile_base_description,
    get_type,
    get_wheelbase_or,
    get_inertia,
    get_command_limits,
)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_previous_robot_namespace(context):
    return LaunchConfiguration("previous_robot_namespace").perform(context)


def get_current_robot_namespace(context):
    return LaunchConfiguration("current_robot_namespace").perform(context)


def get_next_robot_namespace(context):
    return LaunchConfiguration("next_robot_namespace").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    next_robot_namespace = get_next_robot_namespace(context)
    current_robot_namespace = get_current_robot_namespace(context)
    previous_robot_namespace = get_previous_robot_namespace(context)

    platoon = LaunchDescription()

    # platoon.add_action(PushRosNamespace(current_robot_namespace))

    platoon.add_action(SetParameter(name="use_sim_time", value=(mode != "live")))

    platoon.add_action(
        Node(
            package="romea_path_following",
            executable="path_platoon_node",
            name="path_platoon",
            output="screen",
            parameters=[
                {"sampling_period": 0.1},
                {"desired_interdistance":  5.0},
                {"maximal_linear_speed": 1.5},
                {"minimal_linear_acceleration": -1.0},
                {"maximal_linear_acceleration": 1.0},
                {"autoconfigure": True},
                {"autostart": True},
                {"debug": True},
            ],
            remappings=[
                ("previous_vehicle/path_matching/info", "/" + previous_robot_namespace + "/path_matching/info"),
                ("next_vehicle/path_matching/info", "/" + next_robot_namespace + "/path_matching/info"),
            ],
        )
    )
    return [platoon]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("next_robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("current_robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("previous_robot_namespace"))

    # declared_arguments.append(DeclareLaunchArgument("trajectory_file_path"))

    # declared_arguments.append(DeclareLaunchArgument("configuration_file_path"))

    # declared_arguments.append(DeclareLaunchArgument("joystick_meta_description_file_path"))

    # declared_arguments.append(DeclareLaunchArgument("mobile_base_meta_description_file_path"))

    # declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
