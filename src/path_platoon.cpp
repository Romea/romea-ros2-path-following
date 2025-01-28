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
#include <utility>
#include <chrono>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_path_following/path_platoon.hpp"
#include "romea_path_following/path_platoon_parameters.hpp"

using namespace std::chrono_literals;

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
PathPlatoon::PathPlatoon(Node::SharedPtr node)
: node_(std::move(node)),
  is_activated_(false)
{
  declare_sampling_period(node_);
  declare_desired_interdistance(node_);
  declare_maximal_linear_speed(node_);
  declare_maximal_linear_acceleration(node_);
  declare_minimal_linear_acceleration(node_);
  declare_log_directory(node_);
  declare_debug(node_);
}

//-----------------------------------------------------------------------------
void PathPlatoon::configure()
{
  platoon_ = std::make_unique<core::PathFollowingPlatoon>(
    get_sampling_period(node_),
    get_desired_interdistance(node_),
    get_maximal_linear_speed(node_),
    get_minimal_linear_acceleration(node_),
    get_maximal_linear_acceleration(node_));

  if (get_debug(node_)) {
    logger_ = std::make_shared<core::SimpleFileLogger>(get_log_filename(node_));
    platoon_->registerLogger(logger_);
  }

  using namespace std::placeholders;

  auto previous_vehicle_matching_cb = std::bind(
    &PathPlatoon::process_previous_vehicle_matching_info_, this, _1);
  previous_vehicle_matching_sub_ = node_->create_subscription<PathMatchingInfoMsg>(
    "previous_vehicle/path_matching/info", reliable(1), std::move(previous_vehicle_matching_cb));

  auto current_vehicle_matching_cb = std::bind(
    &PathPlatoon::process_current_vehicle_matching_info_, this, _1);
  current_vehicle_matching_sub_ = node_->create_subscription<PathMatchingInfoMsg>(
    "path_matching/info", reliable(1), std::move(current_vehicle_matching_cb));

  auto next_vehicle_matching_cb = std::bind(
    &PathPlatoon::process_next_vehicle_matching_info_, this, _1);
  next_vehicle_matching_sub_ = node_->create_subscription<PathMatchingInfoMsg>(
    "next_vehicle/path_matching/info", reliable(1), std::move(next_vehicle_matching_cb));

  path_following_parameters_client_ = node_->create_client<SetParametersSrv>(
    "/follower/path_following/set_parameters");
}

//-----------------------------------------------------------------------------
void PathPlatoon::activate()
{
  is_activated_ = true;
}

//-----------------------------------------------------------------------------
void PathPlatoon::deactivate()
{
  is_activated_ = false;
}

//-----------------------------------------------------------------------------
void PathPlatoon::process_previous_vehicle_matching_info_(
  PathMatchingInfoMsg::ConstSharedPtr msg)
{
  platoon_->setPreviousVehicleInfo(
    {to_romea_duration(msg->header.stamp),
      to_romea(msg->matched_points),
      to_romea(msg->twist)});
}

//-----------------------------------------------------------------------------
void PathPlatoon::process_current_vehicle_matching_info_(
  PathMatchingInfoMsg::ConstSharedPtr msg)
{
  platoon_->setCurrentVehicleInfo(
      {
        to_romea_duration(msg->header.stamp),
        to_romea(msg->matched_points),
        to_romea(msg->twist)
      }
  );

  auto speed = platoon_->computeLinearSpeedCommand(to_romea_duration(msg->header.stamp));

  if (is_activated_) {
    auto request = std::make_shared<SetParametersSrv::Request>();
    auto parameter = rcl_interfaces::msg::Parameter();
    parameter.name = "setpoint.desired_linear_speed";
    parameter.value.type = 3;
    parameter.value.double_value = speed.value_or(0.5);
    request->parameters.push_back(parameter);
    // std::cout << " send speed " << speed.value_or(0.5) << std::endl;
    path_following_parameters_client_->async_send_request(request);
  }
}

//-----------------------------------------------------------------------------
void PathPlatoon::process_next_vehicle_matching_info_(
  PathMatchingInfoMsg::ConstSharedPtr msg)
{
  platoon_->setNextVehicleInfo(
      {
        to_romea_duration(msg->header.stamp),
        to_romea(msg->matched_points),
        to_romea(msg->twist)
      }
  );
}


}  // namespace ros2
}  // namespace romea
