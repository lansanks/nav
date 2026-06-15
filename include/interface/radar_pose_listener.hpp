#ifndef NAVIGATION_INTERFACE_RADAR_POSE_LISTENER_HPP_
#define NAVIGATION_INTERFACE_RADAR_POSE_LISTENER_HPP_

#include <mutex>
#include <string>

#include "interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navigation
{

class RadarPoseListener
{
public:
  void start(rclcpp::Node & node, const std::string & topic);
  void stop();
  bool active() const;
  bool getState(RobotNavigationState & state) const;

private:
  mutable std::mutex mutex_;
  RobotNavigationState state_;
  std::string topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace navigation

#endif  // NAVIGATION_INTERFACE_RADAR_POSE_LISTENER_HPP_
