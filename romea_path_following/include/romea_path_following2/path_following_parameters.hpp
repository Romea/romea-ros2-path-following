#ifndef __PathFollowingTOTO3_HPP__
#define __PathFollowingTOTO3_HPP__

// ros2
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"
#include "romea_core_path_following/PathFollowingSetPoint.hpp"


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
void declare_inertia(std::shared_ptr<Node> node)
{
  declare_inertia_info(node, "base.inertia");
}

template<typename Node>
core::MobileBaseInertia get_inertia(std::shared_ptr<Node> node)
{
  return get_inertia_info(node, "base.inertia");
}

template<typename CommandLimits, typename Node>
void declare_command_limits(std::shared_ptr<Node> node)
{
  std::cout << "declare_command_limits 2" << std::endl;
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
core::PathFollowingSetPoint get_setpoint(std::shared_ptr<Node> node)
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

}  // namespace ros2
}  // namespace romea

#endif
