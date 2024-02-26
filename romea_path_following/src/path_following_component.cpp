#include "romea_path_following/path_following_component.hpp"

#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_core_mobile_base/info/MobileBaseType.hpp"

namespace romea
{
namespace ros2
{

PathFollowingComponent::PathFollowingComponent(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_following", options))
{
  using std::placeholders::_1;
  node_->register_on_configure(std::bind(&PathFollowingComponent::on_configure, this, _1));
  node_->register_on_activate(std::bind(&PathFollowingComponent::on_activate, this, _1));
  node_->register_on_deactivate(std::bind(&PathFollowingComponent::on_deactivate, this, _1));

  rcl_interfaces::msg::ParameterDescriptor base_type_descr;
  base_type_descr.description = "Type of the robot [4WS4WD, 2FWS2RWD]";
  node_->declare_parameter("base.type", rclcpp::PARAMETER_STRING, std::move(base_type_descr));

  rcl_interfaces::msg::ParameterDescriptor autoconf_descr;
  autoconf_descr.description = "Automatic configuration when the node is created";
  node_->declare_parameter("autoconfigure", false, std::move(autoconf_descr));

  rcl_interfaces::msg::ParameterDescriptor autostart_descr;
  autostart_descr.description = "Automatically start the robot when the node is configured";
  node_->declare_parameter("autostart", false, std::move(autostart_descr));

  auto mobile_base_type = get_paramater<std::string>(node_, "base.type");
  auto command_type = core::get_command_type(mobile_base_type);
  if (command_type == "two_axle_steering") {
    control_ = std::make_unique<PathFollowing<core::TwoAxleSteeringCommand>>(node_);
  } else if (command_type == "one_axle_steering") {
    control_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(node_);
    // } else if (command_type == "skid_steering") {
    // control_ = std::make_unique<PathFollowing<core::OneAxleSteeringCommand>>(node_);
  } else {
    throw std::runtime_error("Mobile base type " + mobile_base_type + " is not supported");
  }

  if (get_parameter<bool>(node_, "autoconfigure")) {
    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }
  }
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_configure(
  const rclcpp_lifecycle::State &)
try
{
  control_->configure();
  RCLCPP_INFO(node_->get_logger(), "configured");
  return CallbackReturn::SUCCESS;

} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("config"), e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_activate(
  const rclcpp_lifecycle::State &)
{
  control_->activate();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PathFollowingComponent::CallbackReturn PathFollowingComponent::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  control_->deactivate();
  return CallbackReturn::SUCCESS;
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::PathFollowingComponent)
