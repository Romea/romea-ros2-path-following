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

#ifndef ROMEA_PATH_FOLLOWING__PATH_PLATOON_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_PLATOON_HPP_

// std
#include <atomic>
#include <functional>
#include <memory>
#include <queue>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


// romea
#include "romea_core_path_following/PathFollowingPlatoon.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"

namespace romea
{
namespace ros2
{


class PathFollowingBase
{
public:
  virtual ~PathFollowingBase() = default;
  virtual void configure() = 0;
  virtual void activate() {}
  virtual void deactivate() {}
};


class PathPlatoon : public PathFollowingBase
{
public:
  using Node = rclcpp_lifecycle::LifecycleNode;
  using PathMatchingInfoMsg = romea_path_msgs::msg::PathMatchingInfo2D;
  using SetParametersSrv = rcl_interfaces::srv::SetParameters;
  using SetParametersClient = rclcpp::Client<SetParametersSrv>;

  // using SetPoint = core::PathFollowingSetPoint;
  // using VehiculeInterface = CommandInterface<CommandType>;
  // using CommandLimits = typename CommandTraits<CommandType>::CommandLimits;
  // using OdometryMeasure = typename CommandTraits<CommandType>::Measure;
  // using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

public:
  explicit PathPlatoon(Node::SharedPtr node);

  virtual ~PathPlatoon() = default;

  void configure() override;

  void activate() override;

  void deactivate() override;

protected:
  void process_previous_vehicle_matching_info_(PathMatchingInfoMsg::ConstSharedPtr msg);

  void process_current_vehicle_matching_info_(PathMatchingInfoMsg::ConstSharedPtr msg);

  void process_next_vehicle_matching_info_(PathMatchingInfoMsg::ConstSharedPtr msg);

protected:
  rclcpp::SubscriptionBase::SharedPtr previous_vehicle_matching_sub_;
  rclcpp::SubscriptionBase::SharedPtr current_vehicle_matching_sub_;
  rclcpp::SubscriptionBase::SharedPtr next_vehicle_matching_sub_;

  std::shared_ptr<Node> node_;
  std::unique_ptr<core::PathFollowingPlatoon> platoon_;
  std::shared_ptr<SetParametersClient> path_following_parameters_client_;
  std::shared_ptr<core::SimpleFileLogger> logger_;
  std::atomic_bool is_activated_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_PLATOON_HPP_
