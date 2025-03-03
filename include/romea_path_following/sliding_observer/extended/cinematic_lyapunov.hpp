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


#ifndef ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__EXTENDED__CINEMATIC_LYAPUNOV_HPP_
#define ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__EXTENDED__CINEMATIC_LYAPUNOV_HPP_

// std
#include <memory>
#include <string>
#include <vector>


// romea
#include "romea_core_path_following/sliding_observer/extended/cinematic_lyapunov.hpp"
#include "romea_path_following/path_following/parameters.hpp"

namespace romea
{
namespace ros2
{
namespace path_following
{

template<typename CommandType>
class SlidingObserverExtendedCinematicLyapunov
  : public core::path_following::SlidingObserverExtendedCinematicLyapunov<CommandType>
{
public:
  using T = SlidingObserverExtendedCinematicLyapunov<CommandType>;
  using Core = core::path_following::SlidingObserverExtendedCinematicLyapunov<CommandType>;
  using Parameters = typename Core::Parameters;

public:
  template<typename Node>
  SlidingObserverExtendedCinematicLyapunov(
    std::shared_ptr<Node> node,
    const std::string & ns = "sliding_observer")
  : Core(
      try_declare_and_get_sampling_period(node),
      try_declare_and_get_wheelbase(node),
      try_declare_and_get_inertia(node),
      std::invoke([node, ns]() {declare_parameters(node, ns); return get_parameters(node, ns);}))
  {
  }

  template<typename Node>
  static void declare_parameters(
    std::shared_ptr<Node> node,
    const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.x_deviation");
    declare_parameter<double>(node, params_ns, "gains.y_deviation");
    declare_parameter<double>(node, params_ns, "gains.course_deviation");
    declare_parameter<double>(node, params_ns, "gains.front_sliding_angle");
    declare_parameter<double>(node, params_ns, "gains.rear_sliding_angle");
  }


  template<typename Node>
  static Parameters get_parameters(
    std::shared_ptr<Node> node,
    const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "gains.x_deviation"),
      get_parameter<double>(node, params_ns, "gains.y_deviation"),
      get_parameter<double>(node, params_ns, "gains.course_deviation"),
      get_parameter<double>(node, params_ns, "gains.front_sliding_angle"),
      get_parameter<double>(node, params_ns, "gains.rear_sliding_angle")
    };
  }

};

}  // namespace path_following
}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__EXTENDED__CINEMATIC_LINEAR_TANGENT_HPP_
