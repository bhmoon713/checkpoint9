#ifndef ATTACH_CLIENT_COMPONENT_HPP_
#define ATTACH_CLIENT_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"

namespace my_components
{

class AttachClient : public rclcpp::Node
{
public:
  explicit AttachClient(const rclcpp::NodeOptions & options);

private:
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
};

}  // namespace attach_shelf

#endif  // ATTACH_CLIENT_COMPONENT_HPP_