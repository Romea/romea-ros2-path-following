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

#ifndef ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CURVATURE_TRANSITION_HPP_
#define ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CURVATURE_TRANSITION_HPP_

// std
#include <string>

// romea
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_path_following/longitudinal_control/curvature_transition.hpp>

namespace romea::ros2::path_following
{

template<typename CommandType>
class LongitudinalControlCurvatureTransition
: public core::path_following::LongitudinalControlCurvatureTransition<CommandType>
{
public:
  using Core = core::path_following::LongitudinalControlCurvatureTransition<CommandType>;
  using Parameters = typename Core::Parameters;

public:
  template<typename Node>
  LongitudinalControlCurvatureTransition(
    std::shared_ptr<Node> node, const std::string & ns = "longitudinal_control")
  : Core(std::invoke([node, ns]() {
      declare_parameters(node, ns);
      return get_parameters(node, ns);
    }))
  {
  }

  template<typename Node>
  static void declare_parameters(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "minimal_linear_speed");
    declare_parameter<double>(node, params_ns, "lateral_error_max");
    declare_parameter<double>(node, params_ns, "settling_time");
    declare_parameter<double>(node, params_ns, "settling_distance");
    declare_parameter<double>(node, params_ns, "convergence_ratio");
  }

  template<typename Node>
  static Parameters get_parameters(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "minimal_linear_speed"),
      get_parameter<double>(node, params_ns, "lateral_error_max"),
      get_parameter<double>(node, params_ns, "settling_time"),
      get_parameter<double>(node, params_ns, "settling_distance"),
      get_parameter<double>(node, params_ns, "convergence_ratio"),
    };
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CURVATURE_TRANSITION_HPP_
