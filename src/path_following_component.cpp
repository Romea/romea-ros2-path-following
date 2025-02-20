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

// std
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_core_mobile_base/info/MobileBaseType.hpp"
#include "romea_path_following/path_following_component.hpp"

namespace romea
{
namespace ros2
{

PathFollowingComponent::PathFollowingComponent(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_following", options))
{
  try {
    using std::placeholders::_1;
    node_->register_on_configure(std::bind(&PathFollowingComponent::on_configure, this, _1));
    node_->register_on_activate(std::bind(&PathFollowingComponent::on_activate, this, _1));
    node_->register_on_deactivate(std::bind(&PathFollowingComponent::on_deactivate, this, _1));

    rcl_interfaces::msg::ParameterDescriptor base_type_descr;
    base_type_descr.description = "Type of the robot [4WS4WD, 2FWS2RWD]";
    node_->declare_parameter("base.type", rclcpp::PARAMETER_STRING, base_type_descr);

    rcl_interfaces::msg::ParameterDescriptor autoconf_descr;
    autoconf_descr.description = "Automatic configuration when the node is created";
    node_->declare_parameter("autoconfigure", false, autoconf_descr);

    rcl_interfaces::msg::ParameterDescriptor autostart_descr;
    autostart_descr.description = "Automatically start the robot when the node is configured";
    node_->declare_parameter("autostart", false, autostart_descr);

    rcl_interfaces::msg::ParameterDescriptor joystick_descr;
    joystick_descr.description = "If enabled, listen joy topic for start/stop actions";
    node_->declare_parameter("enable_joystick", true, joystick_descr);

    declare_joystick_mapping(node_);

    auto mobile_base_type = get_parameter<std::string>(node_, "base.type");
    auto command_type = core::get_command_type(mobile_base_type);
    RCLCPP_INFO_STREAM(node_->get_logger(), "command type: " << command_type);

    if (command_type == "two_axle_steering") {
      control_ = std::make_unique<PathFollowing<core::TwoAxleSteeringCommand>>(node_);
    } else if (command_type == "one_axle_steering") {
      control_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(node_);
    } else if (command_type == "skid_steering") {
      control_ = std::make_unique<PathFollowing<core::SkidSteeringCommand>>(node_);
    } else {
      throw std::runtime_error("Mobile base type " + mobile_base_type + " is not supported");
    }

    if (get_parameter<bool>(node_, "enable_joystick")) {
      joystick_ = std::make_unique<Joystick>(node_, get_joystick_mapping(node_));

      joystick_->registerButtonCallback("start", JoystickButton::PRESSED, [this]() {
        RCLCPP_INFO(node_->get_logger(), "button pressed: start");
        node_->activate();
      });

      joystick_->registerButtonCallback("stop", JoystickButton::PRESSED, [this]() {
        RCLCPP_INFO(node_->get_logger(), "button pressed: stop");
        node_->deactivate();
      });
    }

    if (get_parameter<bool>(node_, "autoconfigure")) {
      auto state = node_->configure();
      if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
        node_->activate();
      }
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("constructor"), e.what());
  }
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_configure(
  const rclcpp_lifecycle::State & /*unused*/)
{
  try {
    control_->configure();
    RCLCPP_INFO(node_->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("config"), e.what());
    return CallbackReturn::FAILURE;
  }
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_activate(
  const rclcpp_lifecycle::State & /*unused*/)
{
  RCLCPP_INFO(node_->get_logger(), "activated");
  control_->activate();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_deactivate(
  const rclcpp_lifecycle::State & /*unused*/)
{
  RCLCPP_INFO(node_->get_logger(), "deactivated");
  control_->deactivate();
  return CallbackReturn::SUCCESS;
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::PathFollowingComponent)
