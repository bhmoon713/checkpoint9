#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/get_direction.hpp"

class RobotPatrolWithService : public rclcpp::Node {
public:
  RobotPatrolWithService() : Node("custom_interfaces_service_node") {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotPatrolWithService::scanCallback, this, std::placeholders::_1));

    client_ = this->create_client<custom_interfaces::srv::GetDirection>("/direction_service");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotPatrolWithService::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), " Robot Patrol with Service Node Started");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan latest_scan_;
  bool scan_received_ = false;

  float min_value_;     // Distance to closest obstacle
  float min_direction_; // Angle to closest obstacle

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
    scan_received_ = true;

    int scanN = msg->ranges.size();
    float left = msg->ranges[scanN * 3 / 4];
    float front = msg->ranges[scanN * 2 / 4];
    float right = msg->ranges[scanN * 1 / 4];
    
    auto min_it = std::min_element(msg->ranges.begin() + scanN * 1.5 / 4,
                                   msg->ranges.begin() + scanN * 2.5 / 4);
    min_value_ = *min_it;
    int min_index = std::distance(msg->ranges.begin(), min_it);
    min_direction_ = (min_index - scanN / 2) * 6.28 / scanN;

    RCLCPP_INFO(this->get_logger(),
                "📡 Laser size: %zu | Left %.2f Front %.2f Right: %.2f",
                msg->ranges.size(), left, front, right);
    }

    void checkvicinity(geometry_msgs::msg::Twist &cmd) {
        if (min_value_ < 0.30 && min_direction_ < 0) {
            cmd.linear.x = 0.05;
            cmd.angular.z = 0.75;
            RCLCPP_INFO(this->get_logger(), "There is something on right");
        } else if (min_value_ < 0.25 && min_direction_ > 0) {
            cmd.linear.x = 0.05;
            cmd.angular.z = -0.75;
            RCLCPP_INFO(this->get_logger(), "There is something on left");
        }

    }

    void timerCallback() {
    if (!scan_received_) return;

    auto cmd = geometry_msgs::msg::Twist();

    // Check front distance
    // int scanN = latest_scan_.ranges.size();
    // float front = latest_scan_.ranges[scanN * 2 / 4];

    // if (front > 0.35) {
    //     cmd.linear.x = 0.1;
    //     cmd.angular.z = 0.0;
    //     RCLCPP_INFO(this->get_logger(), " Path is clear — Going forward");
    //     cmd_pub_->publish(cmd);
    //     return;
    // }

    // Obstacle detected → call service
    if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), " Waiting for /direction_service...");
        return;
    }

    auto request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = latest_scan_;

    RCLCPP_INFO(this->get_logger(), "Sending request to /direction_service");

    // Async request with callback
    client_->async_send_request(request,
        [this](rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture future_response) {
        auto response = future_response.get();
        auto cmd = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), " Direction from service: %s", response->direction.c_str());

        if (response->direction == "forward") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;            
        } else if (response->direction == "left") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.5;
        } else if (response->direction == "right") {
            cmd.linear.x = 0.1;
            cmd.angular.z = -0.5;
        }
        checkvicinity(cmd);
        cmd_pub_->publish(cmd);
        }
    );
    }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrolWithService>());
  rclcpp::shutdown();
  return 0;
}