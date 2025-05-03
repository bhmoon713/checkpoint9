#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

namespace my_components
{

class PreApproach : public rclcpp::Node
{
public:
//   COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions & options);

protected:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timerCallback();
  void getting_params();
  void gotoDist(geometry_msgs::msg::Twist &cmd);
  void turnToShelf(geometry_msgs::msg::Twist &cmd);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time turn_start_time_;
  float front_ = 0.0;
  float obstacle = 0.3 ;
  int degrees = -90 ;
  bool arrived_at_shelf = false;
  bool turning_completed = false;
  bool turning_ = false;
  double turn_duration_sec_ = 10.0;
};

}  // namespace composition

#endif  // COMPOSITION__PRE_APPROACH_COMPONENT_HPP_