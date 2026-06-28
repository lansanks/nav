#ifndef NAVIGATION_APP_NAVIGATION_MOUSE_CONTROLLER_HPP_
#define NAVIGATION_APP_NAVIGATION_MOUSE_CONTROLLER_HPP_

#include "app/navigation_node_context.hpp"
#include "app/navigation_points_workflow.hpp"
#include "app/navigation_ui_coordinator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navigation::app
{

class NavigationMouseController
{
public:
  NavigationMouseController(
    NavigationNodeContext & context,
    NavigationUiCoordinator & ui_coordinator,
    NavigationPointsWorkflow & points_workflow,
    rclcpp::Logger logger);

  void handleMouseEvent(int event, int x, int y, int flags);
  void handleWheelDelta(int x, int y, int delta);

private:
  void handleMouseWheel(int x, int y, int flags);
  void rememberMousePosition(int x, int y);
  void handleMiddleDown(int x, int y);
  void handleMouseMove(int x, int y, int flags);
  void handleMiddleUp();
  void handleLeftClick(int x, int y, int flags);
  void handleRightClick(int x, int y);

  NavigationNodeContext & context_;
  NavigationUiCoordinator & ui_coordinator_;
  NavigationPointsWorkflow & points_workflow_;
  rclcpp::Logger logger_;
  bool map_dragging_{false};
  bool has_last_mouse_position_{false};
  int last_mouse_x_{0};
  int last_mouse_y_{0};
  int drag_last_x_{0};
  int drag_last_y_{0};
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_MOUSE_CONTROLLER_HPP_
