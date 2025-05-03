#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;

namespace my_components
{

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("go_to_loading_client", options)
{
  this->declare_parameter("final_approach", true);
  bool final_approach = this->get_parameter("final_approach").as_bool();

  if (!final_approach) {
    RCLCPP_WARN(this->get_logger(), "❌ final_approach is false — skipping service call.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "✅ final_approach is true — calling service...");

  client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");

  while (!client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for service /approach_shelf...");
  }

  auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Service response: complete = %s", response->complete ? "true" : "false");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service /go_to_loading");
  }
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
