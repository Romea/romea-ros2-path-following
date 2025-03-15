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

#ifndef ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__SKID_SLIDING_HPP_
#define ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__SKID_SLIDING_HPP_

// std
#include <memory>
#include <string>

// romea
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_path_following/lateral_control/skid_sliding.hpp>

#include "romea_path_following/lateral_control/base.hpp"

namespace romea::ros2::path_following
{

template<typename CommandType>
class LateralControlSkidSliding
: public LateralControlBase<core::path_following::LateralControlSkidSliding, CommandType>
{
public:
  using Base = LateralControlBase<core::path_following::LateralControlSkidSliding, CommandType>;
  using Parameters = typename Base::Parameters;
  using Gains = typename Base::Gains;

  using NodeParameter = rclcpp::Parameter;
  using NodeParameters = std::vector<NodeParameter>;

public:
  template<typename Node>
  LateralControlSkidSliding(std::shared_ptr<Node> node, const std::string & ns = "lateral_control")
  : Base(node, ns, std::invoke([node, ns]() {
           declare_parameters(node, ns);
           return get_parameters(node, ns);
         }))
  {
  }

public:
  template<typename Node>
  static void declare_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      declare_parameter<double>(node, parameters_ns, "gains.lateral_kp");
      declare_parameter<double>(node, parameters_ns, "gains.course_kp");
      declare_parameter<double>(node, parameters_ns, "maximal_target_course_deg");
    }
  }

  template<typename Node>
  static Gains get_gains_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      return {
        get_parameter<double>(node, parameters_ns, "gains.lateral_kp"),
        get_parameter<double>(node, parameters_ns, "gains.course_kp"),
      };
    }
  }

  template<typename Node>
  static Parameters get_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      return {
        get_gains_parameters(node, parameters_ns),
        get_parameter<double>(node, parameters_ns, "maximal_target_course_deg") * M_PI / 180.,
      };
    }
  }

private:
  Gains get_gains_(const NodeParameters & node_parameters) override
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      return {
        this->get_gain_(node_parameters, "gains.lateral_kp", this->default_gains_.lateral_kp),
        this->get_gain_(node_parameters, "gains.course_kp", this->default_gains_.course_kp),
      };
    }
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__BACK_STEPPING_HPP_
