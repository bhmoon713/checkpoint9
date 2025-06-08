#include "my_components/attach_server_component.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace my_components
{

AttachServer::AttachServer(const rclcpp::NodeOptions & options)
: Node("approach_server", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diffbot_base_controller/cmd_vel_unstamped", 10);

  service_ = this->create_service<custom_interfaces::srv::GoToLoading>(
    "/approach_shelf",
    std::bind(&AttachServer::handle_approach_request, this, _1, _2));
  elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

  kp_yaw_ = 0.1;
  kp_distance_ = 0.5;

  RCLCPP_INFO(this->get_logger(), "✅ approach_service_server started");
}

void AttachServer::handle_approach_request(
  const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
  std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response)
{
  if (!request->attach_to_shelf) {
    RCLCPP_INFO(this->get_logger(), "📡 Requested only cart_frame publishing. Nothing to do.");
    response->complete = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Final approach: moving toward cart_frame");

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform("cart_frame", "robot_front_laser_base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Waiting for TF cart_frame: %s", ex.what());
      rate.sleep();
      continue;
    }

    double dx = tf.transform.translation.x;
    double dy = tf.transform.translation.y;
    double error_distance = std::sqrt(dx * dx + dy * dy);
    double error_yaw = std::atan2(dy, dx);

    double linear = std::min(0.3, kp_distance_ * error_distance + 0.15);
    double angular = std::clamp(kp_yaw_ * error_yaw, -0.5, 0.5);

    geometry_msgs::msg::Twist cmd;
    if (error_distance < 0.05) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Arrived at cart_frame. Stopping.");
      break;
    }

    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
    rate.sleep();
  }

  geometry_msgs::msg::Twist cmd_forward;
  cmd_forward.linear.x = 0.4;
  for (int i = 0; i < 20; ++i) {
    cmd_vel_pub_->publish(cmd_forward);
    rate.sleep();
  }
  cmd_forward.linear.x = 0.0;
  cmd_vel_pub_->publish(cmd_forward);

  RCLCPP_INFO(this->get_logger(), "Shelf loaded successfully.");

  // 🆙 Publish elevator up command

  std_msgs::msg::String lift_msg;
  lift_msg.data = "up";
  elevator_pub_->publish(lift_msg);
  RCLCPP_INFO(this->get_logger(), "Shelf lifted via /elevator_up");
  response->complete = true;

    // Schedule shutdown 1 second later to avoid crash in component container
    shutdown_timer_ = this->create_wall_timer(
    1s, [this]() {
        RCLCPP_INFO(this->get_logger(), "Shutting down node safely...");
        rclcpp::shutdown();
    });
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
