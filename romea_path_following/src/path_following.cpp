//romea
#include "romea_mobile_base_utils/params/command_interface_parameters.hpp"

// local
#include "romea_path_following2/path_following.hpp"
#include "romea_path_following2/path_following_factory.hpp"


namespace
{
const int XBOX_X_BUTTON = 2;
const int XBOX_B_BUTTON = 1;
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<class CommandType>
PathFollowing<CommandType>::PathFollowing(Node::SharedPtr node)
: node_(std::move(node))
{
  declare_parameter_with_default(node_, "joy_start_button", XBOX_X_BUTTON);
  declare_parameter_with_default(node_, "joy_stop_button", XBOX_B_BUTTON);

  declare_setpoint(node_);
  declare_selected_lateral_control(node_);
  declare_selected_sliding_observer(node_);
  declare_command_limits<CommandLimits>(node);
  declare_command_interface_configuration(node_, "cmd_output");
  //   declare_parameter_with_default(node_, "use_path_velocity", false);
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::configure()
{
  node_->get_parameter("joy_start_button", joy_start_button_id_);
  node_->get_parameter("joy_stop_button", joy_stop_button_id_);

  setpoint_.store(get_setpoint(node_));
  path_following_ = PathFollowingFactory<CommandType>::make(
    node_, get_selected_lateral_control(node_), get_selected_sliding_observer(node_));

  command_limits_.store(get_command_limits<CommandLimits>(node_));
  auto interface_config = get_command_interface_configuration(node_, "cmd_output");
  cmd_interface_ = std::make_unique<VehiculeInterface>(node_, std::move(interface_config));

  using namespace std::placeholders;
  auto matching_cb = std::bind(&PathFollowing::process_matching_info_, this, _1);
  matching_sub_ = node_->create_subscription<romea_path_msgs::msg::PathMatchingInfo2D>(
    "path_matching/info", reliable(1), std::move(matching_cb));

  auto odom_cb = std::bind(&PathFollowing::process_odometry_, this, _1);
  odometry_sub_ = node_->create_subscription<OdometryMeasureMsg>(
    "odometry", reliable(1), std::move(odom_cb));

  auto joystick_cb = std::bind(&PathFollowing::process_joystick_, this, _1);
  joystick_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", reliable(1), std::move(joystick_cb));

}
//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::activate()
{
}

template<class CommandType>
void PathFollowing<CommandType>::deactivate()
{
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void PathFollowing<CommandType>::process_odometry_(const OdometryMeasureMsg & msg)
{
  odometry_measure_.store(to_romea(msg.measure));
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_matching_info_(
  romea_path_msgs::msg::PathMatchingInfo2D::ConstSharedPtr msg)
{
  core::Twist2D filtered_twist = to_romea(msg->twist);
  std::vector<core::PathMatchedPoint2D> matchedPoints = to_romea(msg->matched_points);

  if (cmd_interface_->is_started()) {

    auto command = path_following_->computeCommand(
      setpoint_.load(), command_limits_.load(), matchedPoints[0],
      odometry_measure_.load(), filtered_twist);

    cmd_interface_->send_command(command);
    // logger_->writeRow();
  }
}

//-----------------------------------------------------------------------------
template<class CommandType>
void PathFollowing<CommandType>::process_joystick_(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  if (msg->buttons[joy_start_button_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "start button");
    path_following_->reset();
    cmd_interface_->start();
  }

  if (msg->buttons[joy_stop_button_id_]) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("joystick"), "stop button");
    cmd_interface_->stop(true);
  }
}

template class PathFollowing<core::TwoAxleSteeringCommand>;
template class PathFollowing<core::OneAxleSteeringCommand>;

}  // namespace ros2
}  // namespace romea
