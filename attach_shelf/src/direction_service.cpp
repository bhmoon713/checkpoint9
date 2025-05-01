#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/get_direction.hpp"
#include <cmath>  // for std::isfinite

using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService() : Node("direction_service_node")
  {
    service_ = this->create_service<custom_interfaces::srv::GetDirection>(
      "/direction_service",
      std::bind(&DirectionService::handle_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Direction Service Server Ready");
  }
private:
  rclcpp::Service<custom_interfaces::srv::GetDirection>::SharedPtr service_;
  void handle_service(
    const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request,
    std::shared_ptr<custom_interfaces::srv::GetDirection::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service Requested");

    const auto &ranges = request->laser_data.ranges;
    const auto &intensities = request->laser_data.intensities;
    size_t n = ranges.size();

    // === Find intensity > 8000 ===
    for (size_t i = 0; i < intensities.size(); ++i) {
      if (intensities[i] > 100.0f) {
        RCLCPP_INFO(this->get_logger(), "High intensity at index [%zu]: %f", i, intensities[i]);
      }
    }
    
    std::vector<size_t> high_intensity_indices;
    // Step 1: Collect all high intensity indices
    for (size_t i = 0; i < intensities.size(); ++i) {
    if (intensities[i] > 100.0f) {
        high_intensity_indices.push_back(i);
    }
    }

    // Step 2: Group consecutive indices
    std::vector<std::vector<size_t>> groups;
    if (!high_intensity_indices.empty()) {
    std::vector<size_t> current_group = {high_intensity_indices[0]};

    for (size_t i = 1; i < high_intensity_indices.size(); ++i) {
        if (high_intensity_indices[i] == high_intensity_indices[i-1] + 1) {
        // Still consecutive
        current_group.push_back(high_intensity_indices[i]);
        } else {
        // Gap detected ➔ push current group and start a new one
        groups.push_back(current_group);
        current_group = {high_intensity_indices[i]};
        }
    }
    groups.push_back(current_group); // Push the last group
    }

    // Step 3: Find median index for each group
    for (size_t g = 0; g < groups.size(); ++g) {
    const auto &group = groups[g];
    size_t median_index = group[group.size() / 2];
    float median_distance = ranges[median_index];  // get distance at that index

    RCLCPP_INFO(this->get_logger(), "Group %zu median index: %zu, distance: %.3f meters", g, median_index, median_distance);
    }

    bool two_legs_detected = (groups.size() == 2);
    RCLCPP_INFO(this->get_logger(), "Two legs detected? %s", two_legs_detected ? "true" : "false");

    // Define sector boundaries
    size_t right_start = n * 1 / 4;
    size_t right_end   = n * 5 / 12;
    size_t front_start = n * 5 / 12;
    size_t front_end   = n * 7 / 12;
    size_t left_start  = n * 7 / 12;
    size_t left_end    = n * 3 / 4;



    double total_dist_sec_right = 0.0;
    for (size_t i = right_start; i < right_end; ++i) {
    if (std::isfinite(ranges[i])) {  // excludes both inf and NaN
        total_dist_sec_right += ranges[i];
        }
    }

    double total_dist_sec_front = 0.0;
    for (size_t i = front_start; i < front_end; ++i) {
    if (std::isfinite(ranges[i])) {
        total_dist_sec_front += ranges[i];
        }
    }

    double total_dist_sec_left = 0.0;
    for (size_t i = left_start; i < left_end; ++i) {
    if (std::isfinite(ranges[i])) {
        total_dist_sec_left += ranges[i];
        }
    }

    float front_distance = ranges[n / 2];  // Center ray for obstacle detection

    auto min_it = std::min_element(ranges.begin() + 220, ranges.begin() + 440);
    float min_front = (min_it != ranges.end()) ? *min_it : std::numeric_limits<float>::infinity();


    // RCLCPP_INFO(this->get_logger(),
    //   "front_dist: %.2f | Total (R: %.2f, F: %.2f, L: %.2f)",
    //   front_distance, total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);

    // Decision logic
    if (front_distance < 0.35 || min_front < 0.3) {
    // if (total_dist_sec_front < 0.35 *110  || ranges[165] <0.3 || ranges[495] <0.3) {
    // Obstacle detected — evaluate sectors
    RCLCPP_INFO(this->get_logger(),
                "Obstacle detected | front_distance: %.2f | min_front: %.2f",
                front_distance, min_front);
        if (total_dist_sec_front >= total_dist_sec_left && total_dist_sec_front >= total_dist_sec_right) {
            response->direction = "forward";
        } else if (total_dist_sec_left >= total_dist_sec_front && total_dist_sec_left >= total_dist_sec_right) {
            response->direction = "left";
        } else {
            response->direction = "right";
        }

    RCLCPP_INFO(this->get_logger(), "Chosen direction: %s", response->direction.c_str());
    } else {
    response->direction = "forward";
    RCLCPP_INFO(this->get_logger(), "Path is clear — go forward");
    }
    RCLCPP_INFO(this->get_logger(), "Service Completed");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}