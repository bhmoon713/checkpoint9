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

class ApproachServiceServer : public rclcpp::Node
{
public:
  ApproachServiceServer()
  : Node("approach_server"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

    service_ = this->create_service<custom_interfaces::srv::GoToLoading>(
      "/approach_shelf",
      std::bind(&ApproachServiceServer::handle_approach_request, this, _1, _2));

    kp_yaw_ = 0.1;
    kp_distance_ = 0.5;

    RCLCPP_INFO(this->get_logger(), "✅ approach_service_server started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double kp_yaw_, kp_distance_;

  void handle_approach_request(
    const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
    std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response)
  {
    if (!request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(), "📡 Requested only cart_frame publishing (already handled by another node). Nothing to do.");
      response->complete = true;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "🛻 Final approach: moving toward cart_frame");

    // === Move toward TF (cart_frame) ===
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

      double linear = std::min(0.3, kp_distance_ * error_distance);
      double angular = std::clamp(kp_yaw_ * error_yaw, -0.5, 0.5);

      geometry_msgs::msg::Twist cmd;
      if (error_distance < 0.05) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "✅ Arrived at cart_frame. Stopping.");
        break;
      }

      cmd.linear.x = linear;
      cmd.angular.z = angular;
      cmd_vel_pub_->publish(cmd);
      rate.sleep();
    }

    // === Extra 30cm forward ===
    geometry_msgs::msg::Twist cmd_forward;
    cmd_forward.linear.x = 0.4;
    for (int i = 0; i < 20; ++i) {  // ~0.3m at 0.2m/s for 1.5s
      cmd_vel_pub_->publish(cmd_forward);
      rate.sleep();
    }
    cmd_forward.linear.x = 0.0;
    cmd_vel_pub_->publish(cmd_forward);

    RCLCPP_INFO(this->get_logger(), "✅ Shelf loaded successfully.");
    response->complete = true;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
