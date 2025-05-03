#include "my_components/pre_approach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: Node("PreApproach", options)
{
  
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diffbot_base_controller/cmd_vel_unstamped", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PreApproach::timerCallback, this));
}

void PreApproach::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int scanN = msg->ranges.size();
  front_ = msg->ranges[scanN * 3 / 6];
}

void PreApproach::gotoDist(geometry_msgs::msg::Twist &cmd)
{
  if (front_ < 0.05 || !std::isfinite(front_)) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Laser data invalid (front = %.2f), waiting for good scan...", front_);
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return;
  }

  float distdiff = front_ - obstacle;

  if (front_ > obstacle) {
    cmd.linear.x = 0.3 * distdiff / (distdiff + 1) + 0.2;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "🚀 Moving forward | Front: %.2f m (Target: %.2f m)", front_, obstacle);
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    arrived_at_shelf = true;
    RCLCPP_INFO(this->get_logger(), "✅ Arrived at target distance. Switching to turning.");
  }
}

void PreApproach::turnToShelf(geometry_msgs::msg::Twist &cmd)
{
  if (!turning_) {
    turning_ = true;
    turn_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "🧭 Start turning for %.1f seconds", turn_duration_sec_);
  }

  rclcpp::Duration elapsed = this->now() - turn_start_time_;
  if (elapsed.seconds() < turn_duration_sec_) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 1.6 * ((degrees * M_PI / 180.0) / turn_duration_sec_);
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    turning_completed = true;
    RCLCPP_INFO(this->get_logger(), "✅ Finished turning after %.1f seconds", turn_duration_sec_);
  }
}

void PreApproach::timerCallback()
{
  auto cmd = geometry_msgs::msg::Twist();

  if (!arrived_at_shelf) {
    gotoDist(cmd);
  } else if (!turning_completed) {
    turnToShelf(cmd);
  } else {
    RCLCPP_INFO(this->get_logger(), "🏁 Turning completed. Ready for call from c...");
  }

  cmd_pub_->publish(cmd);
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)