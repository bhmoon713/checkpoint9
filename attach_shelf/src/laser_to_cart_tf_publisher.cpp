#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class LaserToCartTFPublisher : public rclcpp::Node
{
public:
  LaserToCartTFPublisher()
  : Node("laser_to_cart_tf_publisher")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LaserToCartTFPublisher::scan_callback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto &ranges = msg->ranges;
    const auto &intensities = msg->intensities;

    std::vector<size_t> high_intensity_indices;
    for (size_t i = 0; i < intensities.size(); ++i) {
      if (intensities[i] > 100.0f) {  // Intensity threshold
        high_intensity_indices.push_back(i);
      }
    }

    if (high_intensity_indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No high intensity points found.");
      return;
    }

    // Group consecutive indices
    std::vector<std::vector<size_t>> groups;
    std::vector<size_t> current_group = {high_intensity_indices[0]};
    for (size_t i = 1; i < high_intensity_indices.size(); ++i) {
      if (high_intensity_indices[i] == high_intensity_indices[i-1] + 1) {
        current_group.push_back(high_intensity_indices[i]);
      } else {
        groups.push_back(current_group);
        current_group = {high_intensity_indices[i]};
      }
    }
    groups.push_back(current_group);

    // Only act if exactly 2 groups
    if (groups.size() == 2) {
      // Find median point of each group
      size_t median_index_0 = groups[0][groups[0].size() / 2];
      size_t median_index_1 = groups[1][groups[1].size() / 2];

      float distance_0 = ranges[median_index_0];
      float angle_0 = msg->angle_min + median_index_0 * msg->angle_increment;

      float distance_1 = ranges[median_index_1];
      float angle_1 = msg->angle_min + median_index_1 * msg->angle_increment;

      // Convert to (x, y)
      float x0 = distance_0 * std::cos(angle_0);
      float y0 = distance_0 * std::sin(angle_0);

      float x1 = distance_1 * std::cos(angle_1);
      float y1 = distance_1 * std::sin(angle_1);

      // Calculate the middle point
      float middle_x = (x0 + x1) / 2.0f;
      float middle_y = (y0 + y1) / 2.0f;

      // Publish TF from robot_front_laser_base_link → cart_frame
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "robot_front_laser_base_link";  // From laser
      transform.child_frame_id = "cart_frame";   // To cart frame

      transform.transform.translation.x = middle_x;
      transform.transform.translation.y = middle_y;
      transform.transform.translation.z = 0.0;

      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(transform);

    //   RCLCPP_INFO(this->get_logger(), "Published cart_frame at (x=%.2f, y=%.2f)", middle_x, middle_y);
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Not exactly two groups detected, skipping cart_frame publishing.");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserToCartTFPublisher>());
  rclcpp::shutdown();
  return 0;
}
