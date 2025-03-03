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
#include <string>
#include <utility>

// romea
#include "romea_path_following/external_control/platoon/component.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{
namespace path_following
{

PlatoonComponent::PlatoonComponent(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_platoon", options))
{
  try {
    using std::placeholders::_1;
    node_->register_on_configure(std::bind(&PlatoonComponent::on_configure, this, _1));
    node_->register_on_activate(std::bind(&PlatoonComponent::on_activate, this, _1));
    node_->register_on_deactivate(std::bind(&PlatoonComponent::on_deactivate, this, _1));

    rcl_interfaces::msg::ParameterDescriptor autoconf_descr;
    autoconf_descr.description = "Automatic configuration when the node is created";
    node_->declare_parameter("autoconfigure", false, autoconf_descr);

    rcl_interfaces::msg::ParameterDescriptor autostart_descr;
    autostart_descr.description = "Automatically start the robot when the node is configured";
    node_->declare_parameter("autostart", false, autostart_descr);

    platoon_ = std::make_unique<Platoon>(node_);

    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }

  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("constructor"), e.what());
  }
}

//-----------------------------------------------------------------------------
PlatoonComponent::CallbackReturn PlatoonComponent::on_configure(
  const rclcpp_lifecycle::State &)
{
  // RCLCPP_ERROR_STREAM(node_->get_logger(), "platoon configure !!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  try {
    platoon_->configure();
    RCLCPP_INFO(node_->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("config"), e.what());
    return CallbackReturn::FAILURE;
  }
}

//-----------------------------------------------------------------------------
PlatoonComponent::CallbackReturn PlatoonComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "activated");
  platoon_->activate();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PlatoonComponent::CallbackReturn PlatoonComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "deactivated");
  platoon_->deactivate();
  return CallbackReturn::SUCCESS;
}

}  // namespace path_following
}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::path_following::PlatoonComponent)
