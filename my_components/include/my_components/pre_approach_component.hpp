#ifndef COMPOSITION__PRE_APPROACH_COMPONENT_HPP_
#define COMPOSITION__PRE_APPROACH_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace my_components
{

class PreApproach : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

#endif  // COMPOSITION__PRE_APPROACH_COMPONENT_HPP_