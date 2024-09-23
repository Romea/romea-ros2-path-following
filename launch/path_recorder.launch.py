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
from launch_ros.actions import Node, PushRosNamespace


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_trajectory_file_path(context):
    return LaunchConfiguration("trajectory_file_path").perform(context)


def get_wgs84_anchor(context):
    with open(LaunchConfiguration("wgs84_anchor_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)

    path_recorder = LaunchDescription()

    path_recorder.add_action(PushRosNamespace(robot_namespace))

    latitude = get_wgs84_anchor(context)["latitude"]
    longitude = get_wgs84_anchor(context)["longitude"]
    altitude = get_wgs84_anchor(context)["altitude"]

    path_recorder.add_action(
        Node(
            package="romea_path_tools",
            executable="record",
            name="path_recorder",
            output="screen",
            parameters=[
                {"anchor": [latitude, longitude, altitude]},
                {"output": get_trajectory_file_path(context)},
            ],
            remappings=[("odom", "localisation/filtered_odom")],
        )
    )

    return [path_recorder]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("trajectory_file_path"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
