#ifndef NAVIGATION_APP_NAVIGATION_RUNTIME_HPP_
#define NAVIGATION_APP_NAVIGATION_RUNTIME_HPP_

#include <functional>
#include <string>
#include <vector>

#include "app/navigation_node_context.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navigation::app
{

class NavigationRuntime
{
public:
  using PublishVelocity = std::function<void(const geometry_msgs::msg::Twist &)>;

  NavigationRuntime(
    NavigationNodeContext & context,
    rclcpp::Logger logger,
    PublishVelocity publish_velocity);

  void applyControllerConfig();
  bool isNavigationActive() const;
  void publishZeroVelocity();
  void stopNavigation(const std::string & message);
  void stopNavigationForRouteChange();
  void syncControllerWaypoints();
  void startNavigation(const std::string & controller_name);
  void updateNavigationController(bool has_state, const navigation::RobotNavigationState & state);
  bool handleArmEvent(const std::string & event, std::string * response_message);

private:
  std::vector<navigation::maps::MapPoint> controllerWaypointsForCurrentRace() const;
  void resetMissionTasks();
  void clearMissionPause();
  bool shouldValidateFastMarkers() const;
  bool shouldResumeForEvent(const std::string & event) const;
  bool maybePauseForMissionTask(const navigation::RobotNavigationState & state);
  void sendArrivedToArmIfDue();
  void resumeMissionNavigation(const std::string & reason);
  void sendSetWaypointsRequest(const std::vector<navigation::maps::MapPoint> & points);
  void sendSetConfigRequest(const navigation::ControllerConfig & config);
  void sendStartRequest(
    const std::vector<navigation::maps::MapPoint> & points,
    const navigation::ControllerConfig & config,
    const std::string & controller_name);
  void sendStopRequest(const std::string & reason);

  NavigationNodeContext & context_;
  rclcpp::Logger logger_;
  PublishVelocity publish_velocity_;
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_RUNTIME_HPP_
