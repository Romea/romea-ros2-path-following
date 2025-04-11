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

#ifndef ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_PREDICTIVE_HPP_
#define ROMEA_PATH_FOLLOWING__LATERAL_CONTROL__DESBOS_GENERIC_PREDICTIVE_HPP_

// std
#include <string>

// romea
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_path_following/lateral_control/desbos_generic_predictive.hpp>

// local
#include "romea_path_following/lateral_control/base.hpp"

namespace romea::ros2::path_following
{

template<typename CommandType>
class LateralControlDesbosGenericPredictive
: public LateralControlBase<
    core::path_following::LateralControlDesbosGenericPredictive,
    CommandType>
{
public:
  using Base =
    LateralControlBase<core::path_following::LateralControlDesbosGenericPredictive, CommandType>;
  using Parameters = typename Base::Parameters;
  using Gains = typename Base::Gains;

  using NodeParameter = rclcpp::Parameter;
  using NodeParameters = std::vector<NodeParameter>;

public:
  template<typename Node>
  LateralControlDesbosGenericPredictive(
    std::shared_ptr<Node> node, const std::string & ns = "lateral_control")
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
      declare_parameter<double>(node, parameters_ns, "gains.kp");
      declare_parameter<double>(node, parameters_ns, "gains.kd");
      declare_parameter<double>(node, parameters_ns, "gains.ks");
      declare_parameter<double>(node, parameters_ns, "alpha");
      declare_parameter<double>(node, parameters_ns, "prediction.a0");
      declare_parameter<double>(node, parameters_ns, "prediction.a1");
      declare_parameter<double>(node, parameters_ns, "prediction.b1");
      declare_parameter<double>(node, parameters_ns, "prediction.b2");
      declare_parameter<int>(node, parameters_ns, "prediction.horizon");
      declare_parameter<bool>(node, parameters_ns, "adaptive_gains");
      declare_parameter_with_default(node, parameters_ns, "lmpc", true);
      declare_parameter_with_default<int>(node, parameters_ns, "model_order", 1);
    }
  }

  template<typename Node>
  static Gains get_gains_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      return {
        get_parameter<double>(node, parameters_ns, "gains.kp"),
        get_parameter<double>(node, parameters_ns, "gains.kd"),
        get_parameter<double>(node, parameters_ns, "gains.ks"),
      };
    }
  }

  template<typename Node>
  static Parameters get_parameters(std::shared_ptr<Node> node, const std::string & parameters_ns)
  {
    return {
      get_gains_parameters(node, parameters_ns),
      get_parameter<double>(node, parameters_ns, "alpha"),
      get_parameter<double>(node, parameters_ns, "prediction.a0"),
      get_parameter<double>(node, parameters_ns, "prediction.a1"),
      get_parameter<double>(node, parameters_ns, "prediction.b1"),
      get_parameter<double>(node, parameters_ns, "prediction.b2"),
      get_parameter<int>(node, parameters_ns, "prediction.horizon"),
      get_parameter<bool>(node, parameters_ns, "adaptive_gains"),
      get_parameter<bool>(node, parameters_ns, "lmpc"),
      get_parameter<int>(node, parameters_ns, "model_order"),
    };
  }

private:
  Gains get_gains_(const NodeParameters & node_parameters) override
  {
    if constexpr (std::is_same_v<CommandType, core::SkidSteeringCommand>) {
      return {
        this->get_gain_(node_parameters, "gains.kp", this->default_gains_.kp),
        this->get_gain_(node_parameters, "gains.kd", this->default_gains_.kd),
        this->get_gain_(node_parameters, "gains.ks", this->default_gains_.ks),
      };
    }
  }
};

template<typename CommandType>
class LateralControlDesbosGenericPredictiveHmpc
: public LateralControlDesbosGenericPredictive<CommandType>
{
public:
  template<typename Node>
  LateralControlDesbosGenericPredictiveHmpc(
    std::shared_ptr<Node> node, const std::string & ns = "lateral_control")
  : LateralControlDesbosGenericPredictive<CommandType>(node, ns)
  {
    this->lateral_control_->set_lmpc(false);
  }
};


template<typename CommandType>
class LateralControlDesbosGenericPredictiveLmpc
: public LateralControlDesbosGenericPredictive<CommandType>
{
public:
  template<typename Node>
  LateralControlDesbosGenericPredictiveLmpc(
    std::shared_ptr<Node> node, const std::string & ns = "lateral_control")
  : LateralControlDesbosGenericPredictive<CommandType>(node, ns)
  {
    this->lateral_control_->set_lmpc(true);
  }
};

}  // namespace romea::ros2::path_following

#endif
