// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_

// std
#include <memory>
#include <map>
#include <string>

// ros2
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"
#include "romea_core_path_following/setpoint.hpp"


namespace romea
{
namespace ros2
{


template<typename Node>
void declare_sampling_period(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "sampling_period");
}

template<typename Node>
double get_sampling_period(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "sampling_period");
}

template<typename Node>
double try_declare_and_get_sampling_period(std::shared_ptr<Node> node)
{
  try {
    declare_sampling_period(node);
  } catch (...) {
  }

  return get_sampling_period(node);
}


template<typename Node>
void declare_one_steering_equivalence(std::shared_ptr<Node> node)
{
  declare_parameter_with_default<bool>(node, "one_steering_equivalence", false);
}

template<typename Node>
bool get_one_steering_equivalence(std::shared_ptr<Node> node)
{
  return get_parameter<bool>(node, "one_steering_equivalence");
}


template<typename Node>
void declare_base_type(std::shared_ptr<Node> node)
{
  declare_parameter<std::string>(node, "base.type");
}

template<typename Node>
double get_base_type(std::shared_ptr<Node> node)
{
  return get_parameter<std::string>(node, "base.type");
}

template<typename Node>
void declare_wheelbase(std::shared_ptr<Node> node)
{
  declare_parameter_with_default<double>(node, "base.wheelbase", 1.2);
}

template<typename Node>
double get_wheelbase(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "base.wheelbase");
}

template<typename Node>
double try_declare_and_get_wheelbase(std::shared_ptr<Node> node)
{
  try {
    declare_wheelbase(node);
  } catch (...) {
  }

  return get_wheelbase(node);
}


template<typename Node>
void declare_inertia(std::shared_ptr<Node> node)
{
  declare_inertia_info(node, "base.inertia");
}

template<typename Node>
core::MobileBaseInertia get_inertia(std::shared_ptr<Node> node)
{
  return get_inertia_info(node, "base.inertia");
}

template<typename Node>
core::MobileBaseInertia try_declare_and_get_inertia(std::shared_ptr<Node> node)
{
  try {
    declare_inertia(node);
  } catch (...) {
  }

  return get_inertia(node);
}


template<typename CommandLimits, typename Node>
void declare_command_limits(std::shared_ptr<Node> node)
{
  declare_command_limits<CommandLimits>(node, "base.command_limits");
}

template<typename CommandLimits, typename Node>
CommandLimits get_command_limits(std::shared_ptr<Node> node)
{
  return get_command_limits<CommandLimits>(node, "base.command_limits");
}


template<typename Node>
void declare_setpoint(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "setpoint.desired_linear_speed");
  declare_parameter_with_default<double>(node, "setpoint.desired_lateral_deviation", 0.0);
  declare_parameter_with_default<double>(node, "setpoint.desired_course_deviation", 0.0);
}

template<typename Node>
core::path_following::SetPoint get_setpoint(std::shared_ptr<Node> node)
{
  return{
    get_parameter<double>(node, "setpoint.desired_linear_speed"),
    get_parameter<double>(node, "setpoint.desired_lateral_deviation"),
    get_parameter<double>(node, "setpoint.desired_course_deviation"),
  };
}

template<typename Node>
void declare_selected_lateral_control(std::shared_ptr<Node> node)
{
  declare_parameter<std::string>(node, "lateral_control", "selected");
}

template<typename Node>
std::string get_selected_lateral_control(std::shared_ptr<Node> node)
{
  return get_parameter<std::string>(node, "lateral_control", "selected");
}

template<typename Node>
void declare_selected_sliding_observer(std::shared_ptr<Node> node)
{
  declare_parameter_with_default<std::string>(node, "sliding_observer", "selected", "none");
}

template<typename Node>
std::string get_selected_sliding_observer(std::shared_ptr<Node> node)
{
  return get_parameter<std::string>(node, "sliding_observer", "selected");
}

template<typename Node>
void declare_joystick_start_button_mapping(std::shared_ptr<Node> node)
{
  declare_parameter<int>(node, "joystick", "start");
}

template<typename Node>
void declare_joystick_stop_button_mapping(std::shared_ptr<Node> node)
{
  declare_parameter<int>(node, "joystick", "stop");
}

template<typename Node>
void declare_joystick_mapping(std::shared_ptr<Node> node)
{
  declare_joystick_start_button_mapping(node);
  declare_joystick_stop_button_mapping(node);
}

template<typename Node>
int get_joystick_start_button_mapping(std::shared_ptr<Node> node)
{
  return get_parameter<int>(node, "joystick", "start");
}

template<typename Node>
int get_joystick_stop_button_mapping(std::shared_ptr<Node> node)
{
  return get_parameter<int>(node, "joystick", "stop");
}

template<typename Node>
std::map<std::string, int> get_joystick_mapping(std::shared_ptr<Node> node)
{
  return {
    {"start", get_joystick_start_button_mapping(node)},
    {"stop", get_joystick_stop_button_mapping(node)},
  };
}

template<typename Node>
void declare_stop_at_the_end(std::shared_ptr<Node> node)
{
  declare_parameter_with_default<bool>(node, "stop_at_the_end", true);
}

template<typename Node>
double get_stop_at_the_end(std::shared_ptr<Node> node)
{
  return get_parameter<bool>(node, "stop_at_the_end");
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_PARAMETERS_HPP_
