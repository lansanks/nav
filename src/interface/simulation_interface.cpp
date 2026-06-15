#include "interface.hpp"

#include <cmath>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"

namespace navigation
{
namespace
{

double yawFromQuaternionWxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

class SimulationNavigationInterface final : public NavigationInterface
{
public:
  void start(rclcpp::Node & node) override
  {
    const auto topic = node.declare_parameter<std::string>("sim_odom_topic", "/odom");
    subscription_ = node.create_subscription<nav_msgs::msg::Odometry>(
      topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        RobotNavigationState next;
        next.valid = true;
        next.stamp = rclcpp::Time(msg->header.stamp);
        next.frame_id = msg->header.frame_id;
        next.source = sourceName();

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

    RCLCPP_INFO(
      node.get_logger(),
      "Simulation navigation interface subscribed to '%s'. Start mujoco_runner with publish_odom:=true.",
      topic.c_str());
  }

  bool getState(RobotNavigationState & state) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.valid) {
      return false;
    }
    state = state_;
    return true;
  }

  std::string sourceName() const override
  {
    return "simulation";
  }

private:
  mutable std::mutex mutex_;
  RobotNavigationState state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace

std::unique_ptr<NavigationInterface> createSimulationInterface()
{
  return std::make_unique<SimulationNavigationInterface>();
}

}  // namespace navigation
