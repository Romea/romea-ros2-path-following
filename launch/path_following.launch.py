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


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_trajectory_file_path(context):
    return LaunchConfiguration("trajectory_file_path").perform(context)


def get_wgs84_anchor(context):
    with open(LaunchConfiguration("wgs84_anchor_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_configuration(context):
    with open(LaunchConfiguration("configuration_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_mobile_base_meta_description(context):
    return romea_mobile_base_bringup.load_meta_description(
        LaunchConfiguration("mobile_base_meta_description_file_path").perform(context)
    )


def get_joystick_meta_description(context):
    return romea_joystick_bringup.load_meta_description(
        LaunchConfiguration("joystick_meta_description_file_path").perform(context)
    )


def get_joystick_remapping(joystick_type):

    joystick_remapping_file_path = (
        get_package_share_directory("romea_path_following") +
        "/config/joystick/" + joystick_type + ".yaml"
    )

    with open(joystick_remapping_file_path) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    mobile_base_meta_description = get_mobile_base_meta_description(context)
    joystick_meta_description = get_joystick_meta_description(context)

    joystick_name = joystick_meta_description.get_name()

    print(joystick_meta_description.get_type())
    print(joystick_meta_description.get_driver_pkg())
    joystick_mapping = joystick_buttons_remapping(
        joystick_meta_description.get_type(),
        joystick_meta_description.get_driver_pkg(),
        get_joystick_remapping(joystick_meta_description.get_type()),
    )

    mobile_base_name = mobile_base_meta_description.get_name()

    mobile_base_description = get_mobile_base_description(
        mobile_base_meta_description.get_type(), mobile_base_meta_description.get_model()
    )

    configuration = dict(get_configuration(context)),
    if "replay" in mode:
        configuration["cmd_output"]["priority"] = -1

    # print("config1 type" ,type(configuration))  
    # print("config2 type", type(get_configuration(context)))

    path_following = LaunchDescription()

    path_following.add_action(PushRosNamespace(robot_namespace))

    path_following.add_action(SetParameter(name="use_sim_time", value=(mode != "live")))

    path_following.add_action(
        Node(
            package="romea_path_matching",
            executable="path_matching_node",
            name="path_matching",
            output="screen",
            parameters=[
                {"wgs84_anchor": get_wgs84_anchor(context)},
                {"path": get_trajectory_file_path(context)},
                {"prediction_time_horizon": 1.0},
                {"path_frame_id": "map"},
                {"autoconfigure": True},
                {"autostart": True},
            ],
            remappings=[("odom", "localisation/filtered_odom")],
        )
    )

    path_following.add_action(
        Node(
            package="romea_path_following",
            executable="path_following_node",
            name="path_following",
            output="screen",
            parameters=[
                get_configuration(context),
                {"joystick": joystick_mapping},
                {"base.type": get_type(mobile_base_description)},
                {"base.wheelbase": get_wheelbase_or(mobile_base_description, 1.2)},
                {"base.inertia": get_inertia(mobile_base_description)},
                {"base.command_limits": get_command_limits(mobile_base_description)},
                {"autoconfigure": True},
                {"autostart": True},
            ],
            remappings=[
                ("cmd_mux/subscribe", mobile_base_name + "/cmd_mux/subscribe"),
                ("cmd_mux/unsubscribe", mobile_base_name + "/cmd_mux/unsubscribe"),
                ("odometry", mobile_base_name + "/controller/odometry"),
                ("joystick/joy", joystick_name + "/joy"),
            ],
        )
    )
    return [path_following]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("trajectory_file_path"))

    declared_arguments.append(DeclareLaunchArgument("configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("joystick_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("mobile_base_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
