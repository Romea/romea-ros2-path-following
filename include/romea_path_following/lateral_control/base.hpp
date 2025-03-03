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


#ifndef ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__BASE_HPP_
#define ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__BASE_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// romea
#include "romea_path_following/path_following/parameters.hpp"


namespace romea
{
namespace ros2
{
namespace path_following
{

template<template<typename> class LateralControl, typename CommandType>
class LateralControlBase
{
public:
  using Command = CommandType;
  using LateralContolCore = LateralControl<CommandType>;
  using Parameters = typename LateralContolCore::Parameters;
  using Gains = typename LateralContolCore::Gains;
  using Slidings = typename LateralContolCore::Slidings;
  using CommandLimits = typename LateralContolCore::CommandLimits;
  using OdometryMeasure = typename LateralContolCore::OdometryMeasure;
  using SetPoint = romea::core::path_following::SetPoint;
  using PathFrenetPose2D = romea::core::PathFrenetPose2D;
  using PathPosture2D = romea::core::PathPosture2D;

  using NodeParameter = rclcpp::Parameter;
  using NodeParameters = std::vector<NodeParameter>;
  using OnParametersSetResult = rcl_interfaces::msg::SetParametersResult;
  using OnParametersSetCallbackHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;

public:
  template<typename Node>
  LateralControlBase(
    std::shared_ptr<Node> node,
    const std::string parameters_ns,
    const Parameters & parameters)
  : parameters_ns_(parameters_ns),
    default_gains_(parameters.gains)
  {
    lateral_control_ = std::make_unique<LateralContolCore>(
      try_declare_and_get_sampling_period(node),
      try_declare_and_get_wheelbase(node),
      try_declare_and_get_inertia(node),
      parameters
    );

    set_gains_callback_handle_ = node->add_on_set_parameters_callback(
      std::bind(&LateralControlBase::update_gains_, this, std::placeholders::_1));

  }


  CommandType compute_command(
    const SetPoint & set_point,
    const CommandLimits & command_limits,
    const PathFrenetPose2D & frenet_pose,
    const PathPosture2D & path_posture,
    const double & future_path_curvature,
    const OdometryMeasure & odometry_measure,
    const Slidings & slidings = {})
  {
    return lateral_control_->compute_command(
      set_point,
      command_limits,
      frenet_pose,
      path_posture,
      future_path_curvature,
      odometry_measure,
      slidings);
  }

  void log(romea::core::SimpleFileLogger & logger)
  {
    lateral_control_->log(logger);
  }

  void reset()
  {
    lateral_control_->gains.store(default_gains_);
    lateral_control_->reset();
  }

  Gains get_gains() const
  {
    return lateral_control_->gains.load();
  }

protected:
  double get_gain_(
    const NodeParameters & node_parameters,
    const std::string & name,
    const double & default_value)
  {
    auto full_name = full_param_name(parameters_ns_, name);
    return get_parameter_value_or<double>(node_parameters, full_name, default_value);
  }

  virtual Gains get_gains_(const NodeParameters & node_parameters) = 0;

private:
  OnParametersSetResult update_gains_(const NodeParameters & parameters)
  {
    lateral_control_->gains.store(get_gains_(parameters));
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

protected:
  std::string parameters_ns_;
  std::unique_ptr<LateralContolCore> lateral_control_;

  Gains default_gains_;
  std::shared_ptr<OnParametersSetCallbackHandle> set_gains_callback_handle_;
};

}  // namespace path_following
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__BASE_HPP_
