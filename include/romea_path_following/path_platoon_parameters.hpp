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

#ifndef ROMEA_PATH_FOLLOWING__PATH_PLATOON_PARAMETERS_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_PLATOON_PARAMETERS_HPP_

// std
#include <memory>
#include <map>
#include <string>

// ros2
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_common_utils/params/node_parameters.hpp"


namespace romea
{
namespace ros2
{


template<typename Node>
void declare_desired_linear_speed(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "desired_linear_speed");
}

template<typename Node>
double get_desired_linear_speed(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "desired_linear_speed");
}

template<typename Node>
void declare_deactivated_linear_speed(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "deactivated_linear_speed");
}

template<typename Node>
double get_deactivated_linear_speed(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "deactivated_linear_speed");
}

template<typename Node>
void declare_maximal_linear_speed(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "maximal_linear_speed");
}

template<typename Node>
double get_maximal_linear_speed(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "maximal_linear_speed");
}

template<typename Node>
void declare_maximal_linear_acceleration(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "maximal_linear_acceleration");
}

template<typename Node>
double get_maximal_linear_acceleration(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "maximal_linear_acceleration");
}

template<typename Node>
void declare_minimal_linear_acceleration(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "minimal_linear_acceleration");
}

template<typename Node>
double get_minimal_linear_acceleration(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "minimal_linear_acceleration");
}

template<typename Node>
void declare_desired_interdistance(std::shared_ptr<Node> node)
{
  declare_parameter<double>(node, "desired_interdistance");
}

template<typename Node>
double get_desired_interdistance(std::shared_ptr<Node> node)
{
  return get_parameter<double>(node, "desired_interdistance");
}

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

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_PLATOON_PARAMETERS_HPP_
