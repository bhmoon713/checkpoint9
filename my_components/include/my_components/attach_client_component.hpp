#ifndef ATTACH_CLIENT_COMPONENT_HPP_
#define ATTACH_CLIENT_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"

namespace my_components
{

class AttachClient : public rclcpp::Node
{
public:
  explicit AttachClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void call_service();
  void check_response();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture future_;
};

}  // namespace my_components

#endif  // ATTACH_CLIENT_COMPONENT_HPP_
