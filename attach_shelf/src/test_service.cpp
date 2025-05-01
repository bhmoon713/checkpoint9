#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/get_direction.hpp"

class TestServiceClient : public rclcpp::Node {
public:
  TestServiceClient() : Node("test_service") {
    client_ = this->create_client<custom_interfaces::srv::GetDirection>("/direction_service");

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&TestServiceClient::laser_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), " Service Client Ready");
  }

private:
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /direction_service...");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Service Request");

    auto request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Asynchronous call with response callback
    client_->async_send_request(
      request,
      [this](rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture response_future) {
        auto response = response_future.get();
        RCLCPP_INFO(this->get_logger(), " Service Response: %s", response->direction.c_str());
      }
    );
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestServiceClient>());
  rclcpp::shutdown();
  return 0;
}