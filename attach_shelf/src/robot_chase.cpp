#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using namespace std::chrono_literals;
class RobotChase : public rclcpp::Node
{
public:
  RobotChase() : Node("robot_chase"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RobotChase::chase_callback, this));

    kp_yaw_ = 0.1;
    kp_distance_ = 0.5;
    arrived_cart_frame_ = false;  // Initialize as false
  }

private:
  void chase_callback()
  {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform("cart_frame", "robot_front_laser_base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    double dx = transform.transform.translation.x;
    double dy = transform.transform.translation.y;
    double error_distance = std::sqrt(dx * dx + dy * dy);
    double error_yaw = std::atan2(dy, dx);

    double linear = kp_distance_ * error_distance;
    double angular = kp_yaw_ * error_yaw;

    // Limit the speeds
    linear = std::min(0.5, linear);
    angular = std::clamp(angular, -0.5, 0.5);

    // Stop condition
    // Stop condition
    if (error_distance < 0.05)  // Safe distance threshold
    {
      linear = 0.0;
      angular = 0.0;
      arrived_cart_frame_ = true;  // ✅ Mark as arrived when stopped
      RCLCPP_WARN(this->get_logger(), "Arrived at cart_frame");
    }
    else
    {
      arrived_cart_frame_ = false; // ✅ Otherwise keep it false
    }
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    cmd_vel_pub_->publish(cmd); 
    if (arrived_cart_frame_)
    {
    RCLCPP_INFO(this->get_logger(), "Arrived at cart_frame. Shutting down node...");
    rclcpp::shutdown();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double kp_yaw_;
  double kp_distance_;
  bool arrived_cart_frame_;  // ✅ Added here
};
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}
