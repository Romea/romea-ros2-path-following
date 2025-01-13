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

#ifndef ROMEA_PATH_FOLLOWING__PATH_PLATOON_COMPONENT_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_PLATOON_COMPONENT_HPP_

// std
#include <memory>

// ros
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// romea
#include "romea_path_following/path_platoon.hpp"

namespace romea
{
namespace ros2
{

class PathPlatoonComponent
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit PathPlatoonComponent(const rclcpp::NodeOptions & options);

  auto get_node_base_interface() const {return node_->get_node_base_interface();}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<PathPlatoon> platoon_;
  bool autostart_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_COMPONENT_HPP_
