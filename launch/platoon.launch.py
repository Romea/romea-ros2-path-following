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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def get_wgs84_anchor(context):
    with open(LaunchConfiguration("wgs84_anchor_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_configuration(context):
    with open(LaunchConfiguration("configuration_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    next_robot_namespace = LaunchConfiguration("next_robot_namespace").perform(context)
    current_robot_namespace = LaunchConfiguration("current_robot_namespace").perform(context)
    previous_robot_namespace = LaunchConfiguration("previous_robot_namespace").perform(context)
    trajectory_file_path = LaunchConfiguration("trajectory_file_path").perform(context)

    actions = []

    actions.append(PushRosNamespace(current_robot_namespace))
    actions.append(SetParameter(name="use_sim_time", value=(mode != "live")))

    actions.append(
        Node(
            package="romea_path_matching",
            executable="path_matching_node",
            exec_name="matching_next",
            name="path_matching",
            namespace="next_vehicle",
            output="screen",
            parameters=[
                {
                    "wgs84_anchor": get_wgs84_anchor(context),
                    "path": trajectory_file_path,
                    "prediction_time_horizon": 1.0,
                    "path_frame_id": "map",
                    "autoconfigure": True,
                    "autostart": True,
                }
            ],
            remappings=[("odom", f"/{next_robot_namespace}/localisation/filtered_odom")],
        )
    )

    if previous_robot_namespace:
        actions.append(
            Node(
                package="romea_path_matching",
                executable="path_matching_node",
                exec_name="matching_prev",
                name="path_matching",
                namespace="previous_vehicle",
                output="screen",
                parameters=[
                    {
                        "wgs84_anchor": get_wgs84_anchor(context),
                        "path": trajectory_file_path,
                        "prediction_time_horizon": 1.0,
                        "path_frame_id": "map",
                        "autoconfigure": True,
                        "autostart": True,
                    }
                ],
                remappings=[("odom", f"/{previous_robot_namespace}/localisation/filtered_odom")],
            )
        )

    actions.append(
        Node(
            package="romea_path_following",
            executable="path_platoon_node",
            exec_name="platoon",
            name="path_platoon",
            output="screen",
            parameters=[
                {
                    "autoconfigure": True,
                    "autostart": True,
                    "debug": True,
                },
                get_configuration(context)
            ],
        )
    )
    return [GroupAction(actions)]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("mode"),
        DeclareLaunchArgument("trajectory_file_path"),
        DeclareLaunchArgument("configuration_file_path"),
        DeclareLaunchArgument("wgs84_anchor_file_path"),
        DeclareLaunchArgument("current_robot_namespace", default_value="follower"),
        DeclareLaunchArgument("next_robot_namespace", default_value="leader"),
        DeclareLaunchArgument("previous_robot_namespace", default_value=""),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
