#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
class RobotChase : public rclcpp::Node
{
public:
  RobotChase() : Node("robot_chase"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
    // timer_ = this->create_wall_timer(
    //   100ms, std::bind(&RobotChase::chase_callback, this));
    
    service_ = this->create_service<custom_interfaces::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(&RobotChase::handle_approach_request, this, _1, _2));

    kp_yaw_ = 0.1;
    kp_distance_ = 0.5;
    arrived_cart_frame_ = false;  // Initialize as false
  }

private:
  void handle_approach_request(
    const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
    std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response)
  {
    (void)request;

    rclcpp::Rate rate(10);  // 10 Hz loop
    arrived_cart_frame_ = false;

    while (rclcpp::ok() && !arrived_cart_frame_)
    {
      geometry_msgs::msg::TransformStamped transform;
      try
      {
        transform = tf_buffer_.lookupTransform("cart_frame", "robot_front_laser_base_link", tf2::TimePointZero);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        rate.sleep();
        continue;
      }

      double dx = transform.transform.translation.x;
      double dy = transform.transform.translation.y;
      double error_distance = std::sqrt(dx * dx + dy * dy);
      double error_yaw = std::atan2(dy, dx);

      double linear = kp_distance_ * error_distance;
      double angular = kp_yaw_ * error_yaw;

      linear = std::min(0.3, linear);
      angular = std::clamp(angular, -0.5, 0.5);

      if (error_distance < 0.05)
      {
        linear = 0.0;
        angular = 0.0;
        arrived_cart_frame_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ Arrived at cart_frame");
        rclcpp::shutdown();
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear;
      cmd.angular.z = angular;
      cmd_vel_pub_->publish(cmd);

      rate.sleep();
    }

    response->complete = true;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
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
