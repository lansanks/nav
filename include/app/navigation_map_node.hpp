#ifndef NAVIGATION_APP_NAVIGATION_MAP_NODE_HPP_
#define NAVIGATION_APP_NAVIGATION_MAP_NODE_HPP_

#include <memory>

#include "app/navigation_mouse_controller.hpp"
#include "app/navigation_node_context.hpp"
#include "app/navigation_points_workflow.hpp"
#include "app/navigation_runtime.hpp"
#include "app/navigation_ui_coordinator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ui/window_scroll_controller.hpp"

namespace navigation::app
{

class NavigationMapNode final : public rclcpp::Node
{
public:
  NavigationMapNode();

private:
  static void onMouse(int event, int x, int y, int flags, void * userdata);

  void onTimer();
  void applyFullscreenIfNeeded(int frame_width, int frame_height);
  void publishVelocity(const geometry_msgs::msg::Twist & command);
  void handleRemoteState(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleRemoteStatus(const std_msgs::msg::String::SharedPtr msg);
  void handleArmEvent(
    const std::shared_ptr<navigation::srv::StringCommand::Request> request,
    std::shared_ptr<navigation::srv::StringCommand::Response> response);

  NavigationNodeContext context_;
  NavigationRuntime runtime_;
  NavigationPointsWorkflow points_workflow_;
  NavigationUiCoordinator ui_coordinator_;
  NavigationMouseController mouse_controller_;
  std::unique_ptr<navigation::ui::WindowScrollController> scroll_controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr remote_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remote_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscription_;
  rclcpp::CallbackGroup::SharedPtr arm_event_callback_group_;
  rclcpp::Service<navigation::srv::StringCommand>::SharedPtr arm_event_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  int displayed_frame_width_{0};
  int displayed_frame_height_{0};
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_MAP_NODE_HPP_
