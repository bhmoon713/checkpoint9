#ifndef ATTACH_SERVER_COMPONENT_HPP_
#define ATTACH_SERVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "std_msgs/msg/string.hpp"


#include <memory>
#include <cmath>


namespace my_components
{

class AttachServer : public rclcpp::Node
{
public:
  explicit AttachServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_approach_request(
    const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> request,
    std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> response);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  
  double kp_yaw_, kp_distance_;
};

}  // namespace my_components

#endif  // ATTACH_SERVER_COMPONENT_HPP_
