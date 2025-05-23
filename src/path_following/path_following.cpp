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
#include <vector>
#include <utility>

// romea
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_mobile_base_utils/params/command_interface_parameters.hpp"

// local
#include "romea_path_following/path_following/path_following.hpp"
#include "romea_path_following/path_following/factory.hpp"

namespace romea
{
namespace ros2
{
namespace path_following
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowing<CommandType>::PathFollowing(Node::SharedPtr node)
: node_(std::move(node))
{
  declare_setpoint(node_);
  declare_stop_at_the_end(node_);
  declare_selected_lateral_control(node_);
  declare_selected_sliding_observer(node_);
  declare_command_limits<CommandLimits>(node_);
  declare_command_interface_configuration(node_, "cmd_output");
  declare_log_directory(node_);
  declare_debug(node_);
}

//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowing<CommandType>::~PathFollowing()
{
  cmd_interface_->unsubscribe_to_cmd_mux();
  RCLCPP_INFO_STREAM(node_->get_logger(), "end of node");
}
//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::configure()
{
  setpoint_.store(get_setpoint(node_));
  std::cout << "lateral_control: " << get_selected_lateral_control(node_) << std::endl;
  std::cout << "sliding observer: " << get_selected_sliding_observer(node_) << std::endl;

  if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
    declare_one_steering_equivalence(node_);
    path_following_ = PathFollowingFactory<CommandType>::make(
      node_, get_selected_lateral_control(node_),
      get_selected_sliding_observer(node_),
      get_one_steering_equivalence(node_));
  } else {
    path_following_ = PathFollowingFactory<CommandType>::make(
      node_, get_selected_lateral_control(node_), get_selected_sliding_observer(node_)
    );
  }

  path_following_->set_stop_at_the_end(get_stop_at_the_end(node_));

  if (get_debug(node_)) {
    logger_ = std::make_shared<core::SimpleFileLogger>(get_log_filename(node_));
    path_following_->register_logger(logger_);
  }

  command_limits_.store(get_command_limits<CommandLimits>(node_));
  auto interface_config = get_command_interface_configuration(node_, "cmd_output");
  cmd_interface_ = std::make_unique<VehiculeInterface>(node_, std::move(interface_config));

  using namespace std::placeholders;
  auto matching_cb = std::bind(&PathFollowing::process_matching_info_, this, _1);
  matching_sub_ = node_->create_subscription<romea_path_msgs::msg::PathMatchingInfo2D>(
    "path_matching/info", reliable(1), std::move(matching_cb));

  auto odom_cb = std::bind(&PathFollowing::process_odometry_, this, _1);
  odometry_sub_ = node_->create_subscription<OdometryMeasureMsg>(
    "odometry", reliable(1), std::move(odom_cb));

  on_set_sepoint_parameters_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&PathFollowing<CommandType>::update_setpoint_, this, std::placeholders::_1));
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::activate()
{
  path_following_->reset();
  cmd_interface_->start();
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::deactivate()
{
  cmd_interface_->stop(true);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
typename PathFollowing<CommandType>::OnSetParametersResult
PathFollowing<CommandType>::update_setpoint_(const std::vector<rclcpp::Parameter> & parameters)
{
  setpoint_.store(
    {get_parameter_value<double>(parameters, "setpoint.desired_linear_speed"),
      get_parameter_value_or<double>(parameters, "setpoint.desired_lateral_deviation", 0.0),
      get_parameter_value_or<double>(parameters, "setpoint.desired_course_deviation", 0.0)});
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::process_odometry_(const OdometryMeasureMsg & msg)
{
  odometry_measure_.store(to_romea(msg.measure));
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_matching_info_(
  romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg)
{
  core::Twist2D filtered_twist = to_romea(msg->twist);
  std::vector<core::PathMatchedPoint2D> matchedPoints = to_romea(msg->matched_points);

  if (cmd_interface_->is_started()) {
    if (logger_) {
      logger_->addEntry("time", rclcpp::Time(msg->header.stamp).seconds());
    }

    auto command = path_following_->compute_command(
      setpoint_.load(), command_limits_.load(), matchedPoints,
      odometry_measure_.load(), filtered_twist);

    if (command) {
      cmd_interface_->send_command(*command);
    } else {
      cmd_interface_->send_null_command();
    }

    if (logger_) {
      logger_->writeRow();
    }
  }
}

template class PathFollowing<core::TwoAxleSteeringCommand>;
template class PathFollowing<core::OneAxleSteeringCommand>;
template class PathFollowing<core::SkidSteeringCommand>;

}  // namespace path_following
}  // namespace ros2
}  // namespace romea
