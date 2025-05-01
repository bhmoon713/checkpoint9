#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm> // For min/max element
#include <tuple>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {
    
    this->declare_parameter("obstacle", 0.5);
    this->declare_parameter("degrees", 90.0);
    getting_params();
    // === Publishers and Subscribers ===
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz
                                std::bind(&PreApproach::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PreApproach Node Started");
  }

private:
  // === Member Variables ===
  float front_= 0.0;
  float obstacle;
  float degrees;
  bool arrived_at_shelf = false;
  bool turning_completed = false;
  bool turning_ = false;
  double turn_duration_sec_ = 10.0;  // Set how many seconds you want to turn

  // === Callbacks ===
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int scanN = msg->ranges.size();
    front_ = msg->ranges[scanN * 3 / 6];
    }
  void getting_params() {
        obstacle =
            this->get_parameter("obstacle").get_parameter_value().get<float>();
        degrees =
            this->get_parameter("degrees").get_parameter_value().get<float>();
    }

  void gotoDist(geometry_msgs::msg::Twist &cmd) {
        if (front_ < 0.05 || !std::isfinite(front_)) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Laser data invalid (front = %.2f), waiting for good scan...", front_);
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
            cmd.angular.z =  1.6* ((degrees * M_PI / 180.0) / turn_duration_sec_);  // ✅ Correct parentheses and PI
            // RCLCPP_INFO(this->get_logger(), "🔄 Turning | Time left: %.2f sec", turn_duration_sec_ - elapsed.seconds());
            // RCLCPP_INFO(this->get_logger(), "🔄 Turning | Time elapsed: %.2f sec", elapsed.seconds());
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            turning_completed = true;  // ✅ Mark turning completed
            RCLCPP_INFO(this->get_logger(), "✅ Finished turning after %.1f seconds", turn_duration_sec_);
        }
    }

    void timerCallback() {
        auto cmd = geometry_msgs::msg::Twist();

        if (!arrived_at_shelf) {
            gotoDist(cmd);
        } else if (!turning_completed) {
            turnToShelf(cmd);
        } else {
            RCLCPP_INFO(this->get_logger(), "🏁 Turning completed. Shutting down node...");
            rclcpp::shutdown();  // ✅ Shutdown ROS2 node cleanly
        }

        cmd_pub_->publish(cmd);
    }

  // === Publishers and Subscribers ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time turn_start_time_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
