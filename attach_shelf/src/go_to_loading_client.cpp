#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"

using namespace std::chrono_literals;

class GoToLoadingClient : public rclcpp::Node
{
public:
  GoToLoadingClient() : Node("go_to_loading_client")
  {
    this->declare_parameter("final_approach", false);
    bool final_approach = this->get_parameter("final_approach").as_bool();

    if (!final_approach) {
      RCLCPP_WARN(this->get_logger(), "❌ final_approach is false — skipping service call.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "✅ final_approach is true — calling service...");

    client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/go_to_loading");

    // Wait for the service to be available
    while (!client_->wait_for_service(1s) && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service /go_to_loading...");
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

private:
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToLoadingClient>());
  rclcpp::shutdown();
  return 0;
}
