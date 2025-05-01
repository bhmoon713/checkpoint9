#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2() : Node("pre_approach_v2") {
    this->declare_parameter("obstacle", 0.5);
    this->declare_parameter("degrees", 90.0);
    this->declare_parameter("final_approach", true);

    get_params();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PreApproachV2::scanCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproachV2::timerCallback, this));

    client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");
    elevator_pub_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10); 
    RCLCPP_INFO(this->get_logger(), "✅ PreApproachV2 node started.");
  }

private:
  // === Member Variables ===
  float front_ = 0.0;
  float obstacle, degrees;
  bool final_approach_;
  bool arrived_at_shelf = false;
  bool turning_completed = false;
  bool turning_ = false;
  bool service_called = false;
  rclcpp::Time turn_start_time_;
  double turn_duration_sec_ = 10.0;
  bool elevator_sent_ = false;  // Track if we've sent it
  
  // === ROS Handles ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;

  // === Get parameters ===
  void get_params() {
    obstacle = this->get_parameter("obstacle").as_double();
    degrees = this->get_parameter("degrees").as_double();
    final_approach_ = this->get_parameter("final_approach").as_bool();
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int scanN = msg->ranges.size();
    front_ = msg->ranges[scanN * 3 / 6];
  }

  void gotoDist(geometry_msgs::msg::Twist &cmd) {
    if (front_ < 0.05 || !std::isfinite(front_)) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Invalid laser data (front = %.2f)", front_);
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      return;
    }

    float distdiff = front_ - obstacle;
    if (front_ > obstacle) {
      cmd.linear.x = 0.3 * distdiff / (distdiff + 1) + 0.2;
      cmd.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "🚀 Moving forward | Front: %.2f m (Target: %.2f m)", front_, obstacle);
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      arrived_at_shelf = true;
      RCLCPP_INFO(this->get_logger(), "✅ Arrived at target distance. Switching to turning.");
    }
  }

  void turnToShelf(geometry_msgs::msg::Twist &cmd) {
    if (!turning_) {
      turning_ = true;
      turn_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "🧭 Start turning for %.1f seconds", turn_duration_sec_);
    }

    rclcpp::Duration elapsed = this->now() - turn_start_time_;
    if (elapsed.seconds() < turn_duration_sec_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 1.6 * ((degrees * M_PI / 180.0) / turn_duration_sec_);
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      turning_completed = true;
      RCLCPP_INFO(this->get_logger(), "✅ Finished turning after %.1f seconds", turn_duration_sec_);
    }
  }

  void callApproachShelfService() {
    if (service_called) return;

    while (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for /approach_shelf service...");
      if (!rclcpp::ok()) return;
    }

    auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_;

    using ServiceResponseFuture =
      rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture;

    client_->async_send_request(request,
      [this](ServiceResponseFuture result) {
        auto response = result.get();
        if (response->complete) {
          RCLCPP_INFO(this->get_logger(), "✅ Final approach succeeded. Sending elevator up...");

          std_msgs::msg::String msg;
          msg.data = "up";
          elevator_pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "🛗 Elevator UP message sent.");
        } else {
          RCLCPP_WARN(this->get_logger(), "⚠️ Final approach failed. Skipping elevator.");
        }

        rclcpp::shutdown();  // Shutdown after service response and elevator message
      });

    service_called = true;
    RCLCPP_INFO(this->get_logger(), "📡 Called /approach_shelf service");
  }

  void timerCallback() {
    auto cmd = geometry_msgs::msg::Twist();

    if (!arrived_at_shelf) {
      gotoDist(cmd);
    } else if (!turning_completed) {
      turnToShelf(cmd);
    } else if (!service_called) {
      RCLCPP_INFO(this->get_logger(), "🏁 Turning completed. Calling service...");
      callApproachShelfService();
    }

    cmd_pub_->publish(cmd);
  }

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PreApproachV2>();

  // Use MultiThreadedExecutor to allow async service to complete
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  // ❌ DO NOT call rclcpp::shutdown() here

  return 0;  // Let async service shut down inside its callback
}

