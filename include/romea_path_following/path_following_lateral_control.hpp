#ifndef __PathFollowingTOTO1_HPP__
#define __PathFollowingTOTO1_HPP__

// ros2
#include "rclcpp/rclcpp.hpp"

#include "romea_path_following/path_following_parameters.hpp"
#include "romea_core_path_following/lateral_control/LateralControlBackStepping.hpp"
#include "romea_core_path_following/lateral_control/LateralControlClassic.hpp"
#include "romea_core_path_following/lateral_control/LateralControlPredictive.hpp"

namespace romea
{
namespace ros2
{

template<typename LateralControl>
struct PathFollowingLateralControlParameters
{
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>>
{
  using LateralControl =
    core::PathFollowingLateralControlBackStepping<core::SkidSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.kp");
    declare_parameter_with_default<double>(node, params_ns, "gains.ki", 0.0);
    declare_parameter<double>(node, params_ns, "gains.kd");
    declare_parameter<double>(node, params_ns, "maximal_omega_d");
    declare_parameter_with_default<double>(node, params_ns, "maximal_omega_d_integral", 0.0);
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      {get_parameter<double>(node, params_ns, "gains.kp"),
        get_parameter<double>(node, params_ns, "gains.ki"),
        get_parameter<double>(node, params_ns, "gains.kd")},
      get_parameter<double>(node, params_ns, "maximal_omega_d"),
      get_parameter<double>(node, params_ns, "maximal_omega_d_integral"),
    };
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>>
{
  using LateralControl =
    core::PathFollowingLateralControlClassic<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.front_kd");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      {get_parameter<double>(node, params_ns, "gains.front_kd")}
    };
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>>
{
  using LateralControl =
    core::PathFollowingLateralControlClassic<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.front_kd");
    declare_parameter<double>(node, params_ns, "gains.rear_kd");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      {get_parameter<double>(node, params_ns, "gains.front_kd"),
        get_parameter<double>(node, params_ns, "gains.rear_kd")}
    };
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>>
{
  using LateralControl =
    core::PathFollowingLateralControlPredictive<core::OneAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.front_kd");
    declare_parameter<int>(node, params_ns, "prediction.horizon");
    declare_parameter<double>(node, params_ns, "prediction.a0");
    declare_parameter<double>(node, params_ns, "prediction.a1");
    declare_parameter<double>(node, params_ns, "prediction.b1");
    declare_parameter<double>(node, params_ns, "prediction.b2");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      {get_parameter<double>(node, params_ns, "gains.front_kd")},
      get_parameter<int>(node, params_ns, "prediction.horizon"),
      get_parameter<double>(node, params_ns, "prediction.a0"),
      get_parameter<double>(node, params_ns, "prediction.a1"),
      get_parameter<double>(node, params_ns, "prediction.b1"),
      get_parameter<double>(node, params_ns, "prediction.b2")
    };
  }
};

template<>
struct PathFollowingLateralControlParameters<
  core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>>
{
  using LateralControl =
    core::PathFollowingLateralControlPredictive<core::TwoAxleSteeringCommand>;
  using Parameters = LateralControl::Parameters;

  template<typename Node>
  static void declare(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    declare_parameter<double>(node, params_ns, "gains.front_kd");
    declare_parameter<double>(node, params_ns, "gains.rear_kd");
    declare_parameter<int>(node, params_ns, "prediction.horizon");
    declare_parameter<double>(node, params_ns, "prediction.a0");
    declare_parameter<double>(node, params_ns, "prediction.a1");
    declare_parameter<double>(node, params_ns, "prediction.b1");
    declare_parameter<double>(node, params_ns, "prediction.b2");
  }

  template<typename Node>
  static Parameters get(std::shared_ptr<Node> node, const std::string & params_ns)
  {
    return {
      {get_parameter<double>(node, params_ns, "gains.front_kd"),
        get_parameter<double>(node, params_ns, "gains.rear_kd")},
      get_parameter<int>(node, params_ns, "prediction.horizon"),
      get_parameter<double>(node, params_ns, "prediction.a0"),
      get_parameter<double>(node, params_ns, "prediction.a1"),
      get_parameter<double>(node, params_ns, "prediction.b1"),
      get_parameter<double>(node, params_ns, "prediction.b2")
    };
  }
};

template<typename LateralControl, typename Node>
void declare_lateral_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  PathFollowingLateralControlParameters<LateralControl>::declare(node, params_ns);
}

template<typename LateralControl, typename Node>
typename LateralControl::Parameters get_lateral_control_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  return PathFollowingLateralControlParameters<LateralControl>::get(node, params_ns);
}

template<typename LateralControl, typename Node>
std::shared_ptr<LateralControl> make_lateral_control(
  std::shared_ptr<Node> node,
  const std::string & params_ns)
{
  declare_lateral_control_parameters<LateralControl>(node, params_ns);
  return std::make_shared<LateralControl>(
    get_sampling_period(node),
    get_wheelbase(node),
    get_inertia(node),
    get_lateral_control_parameters<LateralControl>(node, params_ns));
}

}  // namespace ros2
}  // namespace romea

#endif


// class DynamicParameterCallbacksHandler
// {
// public:
//   DynamicParameterCallbacksHandler(std::shared_ptr<rclcpp::Node> node)
//   : param_subscriber_(std::make_shared<rclcpp::ParameterEventHandler>(node)),
//     callbacks_()
//   {
//   }

//   void register_callback(
//     const std::string & parameter_name,
//     rclcpp::ParameterEventHandler::ParameterEventCallbackType callback)
//   {
//     callbacks.push_back(param_subscriber_->add_parameter_callback(parameter_name, callback));
//   }

// private:
//   std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
//   std::list<std::shared_ptr<rclcpp::ParameterCallbackHandle>> callbacks_;
// };

// static void register_callbacks(
//   std::shared_ptr<DynamicParameterCallbacksHandler> callbacks_handler,
//   std::shared_ptr<LateralControl> lateral_control)
// {
//   auto front_kd_callback = [lateral_control](const rclcpp::Parameter & p)
//     {
//       lateral_control->setFrontKD(p.as_double());
//     };

//   callbacks_handler->register_callback("lateral_control.gains.front_kd", front_kd_callback);
// }

// static void register_callbacks(
//   std::shared_ptr<DynamicParameterCallbacksHandler> callbacks_handler,
//   std::shared_ptr<LateralControl> lateral_control)
// {
//   auto front_kd_callback = [lateral_control](const rclcpp::Parameter & p)
//     {
//       lateral_control->setFrontKD(p.as_double());
//     };

//   callbacks_handler->register_callback("lateral_control.gains.front_kd", front_kd_callback);

//   auto rear_kd_callback = [lateral_control](const rclcpp::Parameter & p)
//     {
//       lateral_control->setRearKD(p.as_double());
//     };

//   callbacks_handler->register_callback("lateral_control.gains.rear_kd", rear_kd_callback);

// }
