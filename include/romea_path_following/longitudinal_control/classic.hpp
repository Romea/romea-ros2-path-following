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


#ifndef ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CLASSIC_HPP_
#define ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CLASSIC_HPP_

// std
#include <string>


// romea
#include "romea_core_path_following/longitudinal_control/classic.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{
namespace path_following
{

template<typename CommandType>
class LongitudinalControlClassic
  : public core::path_following::LongitudinalControlClassic<CommandType>
{
public:
  using Core = core::path_following::LongitudinalControlClassic<CommandType>;
  using Parameters = typename Core::Parameters;

public:
  template<typename Node>
  LongitudinalControlClassic(
    std::shared_ptr<Node> node,
    const std::string & ns = "longitudinal_control")
  : Core(std::invoke([node, ns]() {declare_parameters(node, ns); return get_parameters(node, ns);}))
  {
  }

  template<typename Node>
  static void declare_parameters(
    std::shared_ptr<Node> node,
    const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "minimal_linear_speed");
  }


  template<typename Node>
  static Parameters get_parameters(
    std::shared_ptr<Node> node,
    const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "minimal_linear_speed"),
    };
  }

};


}  // namespace path_following
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__LONGITUDINAL_CONTROL__CLASSIC_HPP_
