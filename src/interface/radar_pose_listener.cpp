#include "interface/radar_pose_listener.hpp"

#include <cmath>
#include <mutex>

namespace navigation
{
namespace
{

double yawFromQuaternionWxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

void RadarPoseListener::start(rclcpp::Node & node, const std::string & topic)
{
  if (subscription_ != nullptr && topic_ == topic) {
    return;
  }

  topic_ = topic.empty() ? "/Odometry" : topic;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = RobotNavigationState{};
  }

  subscription_ = node.create_subscription<nav_msgs::msg::Odometry>(
    topic_,
    rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      RobotNavigationState next;
      next.valid = true;
      next.stamp = rclcpp::Time(msg->header.stamp);
      next.frame_id = msg->header.frame_id;
      next.source = "radar";

      next.x = msg->pose.pose.position.x;
      next.y = msg->pose.pose.position.y;
      next.z = msg->pose.pose.position.z;
      next.yaw = yawFromQuaternionWxyz(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

      next.linear_x = msg->twist.twist.linear.x;
      next.linear_y = msg->twist.twist.linear.y;
      next.linear_z = msg->twist.twist.linear.z;
      next.angular_z = msg->twist.twist.angular.z;
      next.planar_speed = std::hypot(next.linear_x, next.linear_y);

      std::lock_guard<std::mutex> lock(mutex_);
      state_ = next;
    });

  RCLCPP_INFO(node.get_logger(), "Radar calibration listener subscribed to '%s'.", topic_.c_str());
}

void RadarPoseListener::stop()
{
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = RobotNavigationState{};
}

bool RadarPoseListener::active() const
{
  return subscription_ != nullptr;
}

bool RadarPoseListener::getState(RobotNavigationState & state) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.valid) {
    return false;
  }
  state = state_;
  return true;
}

}  // namespace navigation
