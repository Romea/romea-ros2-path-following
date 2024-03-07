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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_

// std
#include <memory>
#include <string>

// ros2
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include "romea_core_path_following/longitudinal_control/LongitudinalControlClassic.hpp"

namespace romea
{
namespace ros2
{

template<typename LongitudinalControl>
struct PathFollowingLongitudinalControlParameters
{
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>>
{
  using Longitudinal =
    core::PathFollowingLongitudinalControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
    return {};
  }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>>
{
  using Longitudinal =
    core::PathFollowingLongitudinalControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
    return {};
  }
};

template<>
struct PathFollowingLongitudinalControlParameters<
  core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>>
{
  using Longitudinal =
    core::PathFollowingLongitudinalControlClassic<core::SkidSteeringCommand>;
  using Parameters = Longitudinal::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node>/*node*/, const std::string & /*param_ns*/)
  {
    return {};
  }
};

template<typename LongitudinalControl, typename Node>
void declare_longitudinal_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  PathFollowingLongitudinalControlParameters<LongitudinalControl>::declare(node, params_ns);
}

template<typename LongitudinalControl, typename Node>
typename LongitudinalControl::Parameters get_longitudinal_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  return PathFollowingLongitudinalControlParameters<LongitudinalControl>::get(node, params_ns);
}

template<typename LongitudinalControl, typename Node>
std::shared_ptr<LongitudinalControl> make_longitudinal_control(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  declare_longitudinal_control_parameters<LongitudinalControl>(node, params_ns);
  return std::make_shared<LongitudinalControl>(
    get_longitudinal_control_parameters<LongitudinalControl>(node, params_ns));
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_LONGITUDINAL_CONTROL_HPP_
