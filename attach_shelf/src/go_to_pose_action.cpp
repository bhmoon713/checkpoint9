#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "custom_interfaces/action/go_to_pose.hpp"

using GoToPose = custom_interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

class GoToPoseActionServer : public rclcpp::Node
{
public:
  GoToPoseActionServer() : Node("go_to_pose_action_server")
  {
    using namespace std::placeholders;

    goal_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToPoseActionServer::odom_callback, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    action_server_ = rclcpp_action::create_server<GoToPose>(
      this,
      "/go_to_pose",
      std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
      std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
      std::bind(&GoToPoseActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr goal_sub_;

  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D desired_pos_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Action Called: x=%.2f y=%.2f θ=%.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoToPose>)
  {
    RCLCPP_WARN(this->get_logger(), "❌ Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
  {
    std::thread([this, goal_handle]() {
      this->execute(goal_handle);
    }).detach();
  }
double normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}


    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
    const auto goal = goal_handle->get_goal();
    desired_pos_ = goal->goal_pos;

    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    rclcpp::Rate rate(10); // 10 Hz

    const double distance_thresh = 0.03; // meters
    const double angle_thresh = 0.1;   // radians (~3 degrees)

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "❌ Goal canceled mid-execution");
        return;
        }

        // Calculate pose deltas
        double dx = desired_pos_.x - current_pos_.x;
        double dy = desired_pos_.y - current_pos_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        double angle_to_target = normalize_angle(target_angle - current_pos_.theta);
        double desired_angle_rad = desired_pos_.theta *3.14/180; 
        double final_angle_diff = normalize_angle(desired_angle_rad - current_pos_.theta);

        // Normalize angles to [-π, π]
        while (angle_to_target > M_PI) angle_to_target -= 2 * M_PI;
        while (angle_to_target < -M_PI) angle_to_target += 2 * M_PI;
        while (final_angle_diff > M_PI) final_angle_diff -= 2 * M_PI;
        while (final_angle_diff < -M_PI) final_angle_diff += 2 * M_PI;

        // Feedback to client
        feedback->current_pos = current_pos_;
        goal_handle->publish_feedback(feedback);

        geometry_msgs::msg::Twist cmd;

        if (distance > distance_thresh) {
        // Phase 1: turn to face the target first
        if (std::abs(angle_to_target) > angle_thresh) {
            cmd.angular.z = 0.3 * angle_to_target+0.2;
            cmd.linear.x = 0.0; // do not move just rotate
        } else {
            // Phase 2: drive forward toward the target
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.5 * angle_to_target;
        }
        } else {
        // Final alignment at target position
        if (std::abs(final_angle_diff) > angle_thresh) {
            cmd.angular.z = 0.5 * final_angle_diff;
            cmd.linear.x = 0.0;
        } else {
            // Goal complete
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // stop
            break;
        }
        }

        cmd_pub_->publish(cmd);
        rate.sleep();
        RCLCPP_INFO(this->get_logger(), "Action Called: Desired=%.2f inRad=%.2f θ=%.2f",
                desired_pos_.theta, desired_angle_rad, current_pos_.theta);
    }

    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ Goal reached and aligned.");
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPoseActionServer>());
  rclcpp::shutdown();
  return 0;
}
