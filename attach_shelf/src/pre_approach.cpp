#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {
    // === Declare Parameters ===
    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0.0);

    getting_params();

    // === Publishers and Subscribers ===
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);


    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PreApproach::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "⏳ Waiting 2 seconds for laser scan to be ready...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "✅ Laser scan wait finished, starting normal operation.");
    RCLCPP_INFO(this->get_logger(), "✅ PreApproach Node Started");

  }

private:
  // === Member Variables ===
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
//   bool delay_finished_ = false;

  nav_msgs::msg::Odometry current_odom_;

  float left_ = 0.0;
  float front_ = 0.0;
  float right_ = 0.0;
  float obstacle = 0.0;
  float degrees = 0.0;
  

  bool arrived_at_shelf = false;
  bool finished_turning = false;
  double start_yaw = 0.0;

  // === Helper Functions ===
  void getting_params() {
    obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<float>();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
  }

  double getYaw() {
    tf2::Quaternion q(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // === Core Functions ===

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Check if the scan data is valid
    bool valid_scan = false;
    for (auto range : msg->ranges) {
        if (range > 0.05 && range < 10.0) {  // Valid range: 5cm ~ 10m
            valid_scan = true;
            break;
        }
    }

    if (!valid_scan) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "⚠️ Laser scan still invalid (all 0.0 or inf). Skipping...");
        return;
    }
    int scanN = msg->ranges.size();

    // Safer read: if reading is inf or bad, set high default
    left_ = std::isfinite(msg->ranges[scanN * 1 / 6]) ? msg->ranges[scanN * 1 / 6] : 10.0;
    front_ = std::isfinite(msg->ranges[scanN * 3 / 6]) ? msg->ranges[scanN * 3 / 6] : 10.0;
    right_ = std::isfinite(msg->ranges[scanN * 5 / 6]) ? msg->ranges[scanN * 5 / 6] : 10.0;
}

//   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//     int scanN = msg->ranges.size();

//     // Safer read: if reading is inf or bad, set high default
//     left_ = std::isfinite(msg->ranges[scanN * 1 / 6]) ? msg->ranges[scanN * 1 / 6] : 10.0;
//     front_ = std::isfinite(msg->ranges[scanN * 3 / 6]) ? msg->ranges[scanN * 3 / 6] : 10.0;
//     right_ = std::isfinite(msg->ranges[scanN * 5 / 6]) ? msg->ranges[scanN * 5 / 6] : 10.0;
//   }

  void gotoDist(geometry_msgs::msg::Twist &cmd) {
    float distance_left = front_ - obstacle;

    if (front_ > obstacle) { 
      cmd.linear.x = 0.4; // move fast
    } else if (distance_left < 0.1) {
      cmd.linear.x = 0.15; // slow down
    } else if (front_ == 0.0) {
      cmd.linear.x = 0.15; // slow down
    } else {
      // Close enough to stop
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      arrived_at_shelf = true;
      RCLCPP_INFO(this->get_logger(), "✅ Arrived at target distance");
      return;
    }

    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "🚀 Moving forward | Distance left: %.2f m", distance_left);
  }

  void turnToShelf(geometry_msgs::msg::Twist &cmd) {
    double current_yaw = getYaw();
    double yaw_diff = normalizeAngle(current_yaw - start_yaw);
    double target_rad = degrees * M_PI / 180.0; // Convert degrees to radians
    double remaining_angle = fabs(target_rad) - fabs(yaw_diff);

    if (remaining_angle > 0.05) { // more than 3 degrees left
      cmd.linear.x = 0.0;

      // Smooth slow down near target
      if (remaining_angle < 0.3) {
        cmd.angular.z = 0.2;
      } else {
        cmd.angular.z = 0.5;
      }

      cmd_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "🔄 Turning: %.1f / %.1f deg (remaining: %.1f deg)",
                  yaw_diff * 180.0 / M_PI, degrees, remaining_angle * 180.0 / M_PI);
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      finished_turning = true;
      RCLCPP_INFO(this->get_logger(), "✅ Finished turning %.1f degrees", degrees);
    }
  }

  void timerCallback() {
    auto cmd = geometry_msgs::msg::Twist();
    
    RCLCPP_INFO(this->get_logger(),
                "📡 Laser | Left: %.2f Front: %.2f Right: %.2f ",
                left_, front_, right_);

    if (!arrived_at_shelf) {
      gotoDist(cmd);
    } else if (!finished_turning) {
      static bool started_turning = false;
      if (!started_turning) {
        start_yaw = getYaw();
        started_turning = true;
        RCLCPP_INFO(this->get_logger(), "🧭 Start turning from yaw: %.1f deg",
                    start_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), 
            "🔄 Turning: %.1f / %.1f deg (remaining: %.1f deg)", 
            current_angle, target_angle, remaining_angle);
      }
      turnToShelf(cmd);
    }
  }
};

// === Main ===
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
