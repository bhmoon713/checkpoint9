#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;

namespace my_components
{

AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("go_to_loading_client", options)
{
  client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");

  timer_ = this->create_wall_timer(100ms, std::bind(&AttachClient::call_service, this));
}

void AttachClient::call_service()
{
  if (!client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "Service /approach_shelf not available.");
    return;
  }

  auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
  request->attach_to_shelf = true;

  future_ = client_->async_send_request(request);

  // Switch to response-check timer
  timer_->cancel();
  timer_ = this->create_wall_timer(100ms, std::bind(&AttachClient::check_response, this));
}

void AttachClient::check_response()
{
  if (future_.valid() &&
      future_.wait_for(0s) == std::future_status::ready)
  {
    auto response = future_.get();
    RCLCPP_INFO(this->get_logger(), "Service response: complete = %s", response->complete ? "true" : "false");
    timer_->cancel();  // Done
    rclcpp::shutdown();
  }
}

}  // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
