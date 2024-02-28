#ifndef __PathFollowingTOTO2_HPP__
#define __PathFollowingTOTO2_HPP__

// ros2
#include "rclcpp/rclcpp.hpp"

#include "romea_path_following2/path_following_parameters.hpp"
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

#endif
