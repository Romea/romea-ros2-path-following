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

#ifndef ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_
#define ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_

// std
#include <memory>
#include <string>

// ros2
#include "rclcpp/rclcpp.hpp"

#include "romea_path_following/path_following_parameters.hpp"
#include \
  "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLinearTangent.hpp"
#include "romea_core_path_following/sliding_observer/SlidingObserverExtendedCinematicLyapunov.hpp"

namespace romea
{
namespace ros2
{


template<typename Observer>
struct PathFollowingSlidingObserverParameters
{
};


template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::OneAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.lateral_deviation");
    declare_parameter<double>(node, params_ns, "gains.course_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.lateral_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.course_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle");
    declare_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "gains.lateral_deviation"),
      get_parameter<double>(node, params_ns, "gains.course_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.lateral_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.course_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle"),
      get_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::TwoAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLinearTangent<core::TwoAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.lateral_deviation");
    declare_parameter<double>(node, params_ns, "gains.course_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.lateral_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.course_deviation");
    declare_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle");
    declare_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "gains.lateral_deviation"),
      get_parameter<double>(node, params_ns, "gains.course_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.lateral_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.course_deviation"),
      get_parameter<double>(node, params_ns, "filter_weights.front_sliding_angle"),
      get_parameter<double>(node, params_ns, "filter_weights.rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.x_deviation");
    declare_parameter<double>(node, params_ns, "gains.y_deviation");
    declare_parameter<double>(node, params_ns, "gains.course_deviation");
    declare_parameter<double>(node, params_ns, "gains.front_sliding_angle");
    declare_parameter<double>(node, params_ns, "gains.rear_sliding_angle");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "gains.x_deviation"),
      get_parameter<double>(node, params_ns, "gains.y_deviation"),
      get_parameter<double>(node, params_ns, "gains.course_deviation"),
      get_parameter<double>(node, params_ns, "gains.front_sliding_angle"),
      get_parameter<double>(node, params_ns, "gains.rear_sliding_angle")};
  }
};

template<>
struct PathFollowingSlidingObserverParameters<
  core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::TwoAxleSteeringCommand>>
{
  using Observer =
    core::PathFollowingSlidingObserverExtendedCinematicLyapunov<core::OneAxleSteeringCommand>;
  using Parameters = Observer::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.x_deviation");
    declare_parameter<double>(node, params_ns, "gains.y_deviation");
    declare_parameter<double>(node, params_ns, "gains.course_deviation");
    declare_parameter<double>(node, params_ns, "gains.front_sliding_angle");
    declare_parameter<double>(node, params_ns, "gains.rear_sliding_angle");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      get_parameter<double>(node, params_ns, "gains.x_deviation"),
      get_parameter<double>(node, params_ns, "gains.y_deviation"),
      get_parameter<double>(node, params_ns, "gains.course_deviation"),
      get_parameter<double>(node, params_ns, "gains.front_sliding_angle"),
      get_parameter<double>(node, params_ns, "gains.rear_sliding_angle")};
  }
};

template<typename SlidingObserver, typename Node>
void declare_sliding_observer_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  PathFollowingSlidingObserverParameters<SlidingObserver>::declare(node, params_ns);
}

template<typename SlidingObserver, typename Node>
typename SlidingObserver::Parameters get_sliding_observer_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  return PathFollowingSlidingObserverParameters<SlidingObserver>::get(node, params_ns);
}

template<typename SlidingObserver, typename Node>
std::shared_ptr<SlidingObserver> make_sliding_observer(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  declare_sliding_observer_parameters<SlidingObserver>(node, params_ns);
  return std::make_shared<SlidingObserver>(
    get_sampling_period(node),
    get_wheelbase(node),
    get_inertia(node),
    get_sliding_observer_parameters<SlidingObserver>(node, params_ns));
}


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_FOLLOWING__PATH_FOLLOWING_SLIDING_OBSERVER_HPP_
