#ifndef __PathFollowing_HPP__
#define __PathFollowing_HPP__

//std
#include <atomic>
#include <functional>
#include <memory>
#include <queue>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// romea
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_mobile_base_utils/control/command_interface.hpp"
#include "romea_mobile_base_utils/control/command_traits.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"

#include "romea_path_following2/path_following_factory.hpp"

namespace romea
{
namespace ros2
{

class PathFollowingBase
{
public:
  virtual ~PathFollowingBase() = default;
  virtual void configure() = 0;
  virtual void activate() {}
  virtual void deactivate() {}
};


template<class CommandType>
class PathFollowing : public PathFollowingBase
{
public:
  using Node = rclcpp_lifecycle::LifecycleNode;
  using SetPoint = core::PathFollowingSetPoint;
  using VehiculeInterface = CommandInterface<CommandType>;
  using CommandLimits = typename CommandTraits<CommandType>::CommandLimits;
  using OdometryMeasure = typename CommandTraits<CommandType>::Measure;
  using OdometryMeasureMsg = typename CommandTraits<CommandType>::MeasureMsg;

public:
  PathFollowing(Node::SharedPtr node);

  virtual ~PathFollowing() = default;

  void configure() override;

  void activate() override;

  void deactivate() override;

protected:
  void process_matching_info_(romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg);

  void process_odometry_(const OdometryMeasureMsg & msg);

  void process_joystick_(sensor_msgs::msg::Joy::ConstSharedPtr msg);

protected:
  Node::SharedPtr node_;

  std::unique_ptr<VehiculeInterface> cmd_interface_;
  rclcpp::SubscriptionBase::SharedPtr matching_sub_;
  rclcpp::SubscriptionBase::SharedPtr odometry_sub_;
  rclcpp::SubscriptionBase::SharedPtr joystick_sub_;

  core::SharedVariable<SetPoint> setpoint_;
  core::SharedVariable<CommandLimits> command_limits_;
  core::SharedVariable<OdometryMeasure> odometry_measure_;
  std::unique_ptr<core::PathFollowingBase<CommandType>> path_following_;

  int joy_start_button_id_;
  int joy_stop_button_id_;
};

}  // namespace ros2
}  // namespace romea

#endif
