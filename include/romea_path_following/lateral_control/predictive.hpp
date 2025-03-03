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


#ifndef ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_
#define ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "romea_core_path_following/lateral_control/predictive.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_path_following/lateral_control/base.hpp"

namespace romea
{
namespace ros2
{
namespace path_following
{

template<typename CommandType>
class LateralControlPredictive
  : public LateralControlBase<core::path_following::LateralControlPredictive, CommandType>
{
public:
  using Base = LateralControlBase<core::path_following::LateralControlPredictive, CommandType>;
  using Parameters = typename Base::Parameters;
  using Gains = typename Base::Gains;

  using NodeParameter = rclcpp::Parameter;
  using NodeParameters = std::vector<NodeParameter>;

public:
  template<typename Node>
  LateralControlPredictive(
    std::shared_ptr<Node> node,
    const std::string & ns = "lateral_control")
  : Base(node, ns, std::invoke([node, ns]() {
        declare_parameters(node, ns); return get_parameters(node, ns);
      }))
  {
  }

public:
  template<typename Node>
  static void declare_parameters(
    std::shared_ptr<Node> node,
    const std::string & parameters_ns)
  {
    declare_parameter<int>(node, parameters_ns, "prediction.horizon");
    declare_parameter<double>(node, parameters_ns, "prediction.a0");
    declare_parameter<double>(node, parameters_ns, "prediction.a1");
    declare_parameter<double>(node, parameters_ns, "prediction.b1");
    declare_parameter<double>(node, parameters_ns, "prediction.b2");

    if constexpr (std::is_same_v<CommandType, core::OneAxleSteeringCommand>)
    {
      declare_parameter<double>(node, parameters_ns, "gains.front_kd");
    } else if constexpr (std::is_same_v<CommandType, core::TwoAxleSteeringCommand>) {
      declare_parameter<double>(node, parameters_ns, "gains.front_kd");
      declare_parameter<double>(node, parameters_ns, "gains.rear_kd");
    }
  }

  template<typename Node>
  static Gains get_gains_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    if constexpr (std::is_same_v<CommandType, core::OneAxleSteeringCommand>)
    {
      return {
        get_parameter<double>(node, parameters_ns, "gains.front_kd")
      };
    } else if constexpr (std::is_same_v<CommandType, core::TwoAxleSteeringCommand>) {
      return {
        get_parameter<double>(node, parameters_ns, "gains.front_kd"),
        get_parameter<double>(node, parameters_ns, "gains.rear_kd")
      };
    }
  }

  template<typename Node>
  static Parameters get_parameters(
    std::shared_ptr<Node> node,
    const std::string & parameters_ns)
  {
    return {
      get_gains_parameters(node, parameters_ns),
      get_parameter<int>(node, parameters_ns, "prediction.horizon"),
      get_parameter<double>(node, parameters_ns, "prediction.a0"),
      get_parameter<double>(node, parameters_ns, "prediction.a1"),
      get_parameter<double>(node, parameters_ns, "prediction.b1"),
      get_parameter<double>(node, parameters_ns, "prediction.b2")
    };
  }

private:
  Gains get_gains_(const NodeParameters & node_parameters) override
  {
    if constexpr (std::is_same_v<CommandType, core::OneAxleSteeringCommand>)
    {
      return {
        this->get_gain_(node_parameters, "gains.front_kd", this->default_gains_.front_kd)
      };
    } else if constexpr (std::is_same_v<CommandType, core::TwoAxleSteeringCommand>) {
      return {
        this->get_gain_(node_parameters, "gains.front_kd", this->default_gains_.front_kd),
        this->get_gain_(node_parameters, "gains.rear_kd", this->default_gains_.rear_kd)
      };
    }
  }
};

}  // namespace path_following
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__PREDICTIVE_HPP_
