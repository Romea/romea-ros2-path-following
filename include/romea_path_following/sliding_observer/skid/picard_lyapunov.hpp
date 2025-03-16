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

#ifndef ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__SKID__PICARD_LYAPUNOV_HPP_
#define ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__SKID__PICARD_LYAPUNOV_HPP_

// std
#include <memory>
#include <string>

// romea
#include <romea_core_path_following/sliding_observer/skid/picard_lyapunov.hpp>

#include "romea_path_following/path_following/parameters.hpp"

namespace romea::ros2::path_following
{

template<typename CommandType>
class SlidingObserverPicardSkidLyapunov
: public core::path_following::SlidingObserverPicardSkidLyapunov<CommandType>
{
public:
  using T = SlidingObserverPicardSkidLyapunov<CommandType>;
  using Core = core::path_following::SlidingObserverPicardSkidLyapunov<CommandType>;
  using Parameters = typename Core::Parameters;

public:
  template<typename Node>
  SlidingObserverPicardSkidLyapunov(
    std::shared_ptr<Node> node, const std::string & ns = "sliding_observer")
  : Core(try_declare_and_get_sampling_period(node), std::invoke([node, ns]() {
           declare_parameters(node, ns);
           return get_parameters(node, ns);
         }))
  {
  }

  template<typename Node>
  static void declare_parameters(std::shared_ptr<Node> node, const std::string & ns)
  {
    declare_parameter<double>(node, ns, "gains.position_x_kp");
    declare_parameter_with_default(node, ns, "gains.position_x_ki", 0.);
    declare_parameter<double>(node, ns, "gains.position_y_kp");
    declare_parameter<double>(node, ns, "gains.course_kp");
    declare_parameter<double>(node, ns, "gains.slip_angle_kp");
    declare_parameter<double>(node, ns, "gains.angular_speed_disturb_kp");
    declare_parameter<double>(node, ns, "gains.linear_speed_disturb_kp");
  }

  template<typename Node>
  static Parameters get_parameters(std::shared_ptr<Node> node, const std::string & ns)
  {
    return {
      get_parameter<double>(node, ns, "gains.position_x_kp"),
      get_parameter<double>(node, ns, "gains.position_x_ki"),
      get_parameter<double>(node, ns, "gains.position_y_kp"),
      get_parameter<double>(node, ns, "gains.course_kp"),
      get_parameter<double>(node, ns, "gains.slip_angle_kp"),
      get_parameter<double>(node, ns, "gains.angular_speed_disturb_kp"),
      get_parameter<double>(node, ns, "gains.linear_speed_disturb_kp"),
    };
  }
};

}  // namespace romea::ros2::path_following

#endif  // ROMEA_PATH_FOLLOWING__SLIDING_OBSERVER__EXTENDED__CINEMATIC_LINEAR_TANGENT_HPP_
