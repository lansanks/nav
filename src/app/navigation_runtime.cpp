#include "app/navigation_runtime.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "controller.hpp"
#include "maps/navigation_map_helpers.hpp"
#include "navigation/srv/set_waypoints.hpp"
#include "navigation/srv/set_controller_config.hpp"
#include "navigation/srv/start_navigation.hpp"
#include "navigation/srv/stop_navigation.hpp"

namespace navigation::app
{
namespace
{

void fillConfigRequest(
  navigation::srv::SetControllerConfig::Request & request,
  const navigation::ControllerConfig & config)
{
  request.waypoint_tolerance = config.waypoint_tolerance;
  request.max_linear_speed = config.max_linear_speed;
  request.max_angular_speed = config.max_angular_speed;
  request.k_rho = config.k_rho;
  request.k_alpha = config.k_alpha;
  request.k_beta = config.k_beta;
  request.fast_max_linear_speed = config.fast_max_linear_speed;
  request.fast_max_angular_speed = config.fast_max_angular_speed;
  request.fast_k_rho = config.fast_k_rho;
  request.fast_k_alpha = config.fast_k_alpha;
  request.fast_k_beta = config.fast_k_beta;
}

void fillStartConfigRequest(
  navigation::srv::StartNavigation::Request & request,
  const navigation::ControllerConfig & config)
{
  request.waypoint_tolerance = config.waypoint_tolerance;
  request.max_linear_speed = config.max_linear_speed;
  request.max_angular_speed = config.max_angular_speed;
  request.k_rho = config.k_rho;
  request.k_alpha = config.k_alpha;
  request.k_beta = config.k_beta;
  request.fast_max_linear_speed = config.fast_max_linear_speed;
  request.fast_max_angular_speed = config.fast_max_angular_speed;
  request.fast_k_rho = config.fast_k_rho;
  request.fast_k_alpha = config.fast_k_alpha;
  request.fast_k_beta = config.fast_k_beta;
}

std::string normalizeRaceLogic(const std::string & race_logic)
{
  return race_logic == "mission" ? "mission" : "obstacle";
}

template<typename ClientT>
bool serviceReady(
  const typename ClientT::SharedPtr & client,
  NavigationNodeContext & context,
  rclcpp::Logger logger,
  const char * service_name)
{
  if (client == nullptr) {
    context.status_message = std::string(service_name) + " client unavailable";
    RCLCPP_WARN(logger, "%s client is not available.", service_name);
    return false;
  }
  if (!client->service_is_ready()) {
    context.status_message = std::string(service_name) + " service unavailable";
    context.pending_op_start = std::chrono::steady_clock::time_point{};
    RCLCPP_WARN(logger, "%s service is not ready.", service_name);
    return false;
  }
  return true;
}

}  // namespace

NavigationRuntime::NavigationRuntime(
  NavigationNodeContext & context,
  rclcpp::Logger logger,
  PublishVelocity publish_velocity)
: context_(context),
  logger_(logger),
  publish_velocity_(std::move(publish_velocity))
{
}

void NavigationRuntime::applyControllerConfig()
{
  if (context_.remote_control) {
    sendSetConfigRequest(context_.controller_config);
    return;
  }

  if (context_.controller != nullptr) {
    context_.controller->configure(context_.controller_config);
  }
}

bool NavigationRuntime::isNavigationActive() const
{
  if (context_.remote_control) {
    return context_.remote_navigation_active;
  }
  return context_.controller != nullptr && context_.controller->status().active;
}

void NavigationRuntime::publishZeroVelocity()
{
  if (context_.remote_control) {
    return;
  }

  if (publish_velocity_ != nullptr) {
    publish_velocity_(geometry_msgs::msg::Twist{});
  }
}

void NavigationRuntime::stopNavigation(const std::string & message)
{
  if (context_.remote_control) {
    sendStopRequest(message);
    return;
  }

  if (context_.controller != nullptr) {
    context_.controller->stop();
  }
  clearMissionPause();
  publishZeroVelocity();
  context_.navigation_status = message;
  context_.status_message = message;
  RCLCPP_INFO(logger_, "%s", message.c_str());
}

void NavigationRuntime::stopNavigationForRouteChange()
{
  if (isNavigationActive()) {
    stopNavigation("Navigation stopped: route changed");
  }
}

void NavigationRuntime::syncControllerWaypoints()
{
  if (context_.remote_control) {
    sendSetWaypointsRequest(context_.map->points());
    return;
  }

  resetMissionTasks();
  if (context_.controller == nullptr) {
    return;
  }

  context_.controller->setWaypoints(controllerWaypointsForCurrentRace());
  context_.navigation_status = context_.controller->status().message;
}

void NavigationRuntime::startNavigation(const std::string & controller_name)
{
  if (context_.remote_control) {
    std::string marker_error;
    if (shouldValidateFastMarkers() &&
      !navigation::maps::validateFastMarkers(context_.map->points(), &marker_error))
    {
      context_.navigation_status = marker_error;
      context_.status_message = marker_error;
      RCLCPP_WARN(logger_, "%s", marker_error.c_str());
      return;
    }

    context_.selected_controller_name = controller_name;
    sendStartRequest(context_.map->points(), context_.controller_config, controller_name);
    return;
  }

  std::string marker_error;
  if (shouldValidateFastMarkers() &&
    !navigation::maps::validateFastMarkers(context_.map->points(), &marker_error))
  {
    context_.navigation_status = marker_error;
    context_.status_message = marker_error;
    RCLCPP_WARN(logger_, "%s", marker_error.c_str());
    publishZeroVelocity();
    return;
  }

  auto next_controller = navigation::createController(controller_name, context_.controller_config);
  if (next_controller == nullptr) {
    context_.navigation_status = "Controller unavailable";
    context_.status_message = context_.navigation_status;
    RCLCPP_WARN(logger_, "Unknown navigation controller: %s", controller_name.c_str());
    return;
  }

  context_.selected_controller_name = controller_name;
  resetMissionTasks();
  next_controller->setWaypoints(controllerWaypointsForCurrentRace());

  std::string error_message;
  if (!next_controller->start(&error_message)) {
    context_.controller = std::move(next_controller);
    context_.navigation_status = error_message.empty() ? "Navigation start failed" : error_message;
    context_.status_message = context_.navigation_status;
    RCLCPP_WARN(logger_, "%s", context_.navigation_status.c_str());
    publishZeroVelocity();
    return;
  }

  context_.controller = std::move(next_controller);
  context_.navigation_status = context_.controller->status().message;
  context_.status_message = "Navigation started";
  RCLCPP_INFO(logger_, "Navigation started with controller '%s'.", context_.selected_controller_name.c_str());
}

void NavigationRuntime::sendSetWaypointsRequest(
  const std::vector<navigation::maps::MapPoint> & points)
{
  if (!serviceReady<rclcpp::Client<navigation::srv::SetWaypoints>>(
      context_.set_waypoints_client,
      context_,
      logger_,
      "SetWaypoints"))
  {
    return;
  }

  auto request = std::make_shared<navigation::srv::SetWaypoints::Request>();
  for (const auto & point : points) {
    navigation::msg::MapPoint mp;
    mp.id = point.id;
    mp.x = point.x;
    mp.y = point.y;
    mp.fast = point.fast;
    request->points.push_back(mp);
  }
  request->race_logic = normalizeRaceLogic(context_.race_logic);

  context_.status_message = "Sending route to core...";
  context_.pending_op_start = std::chrono::steady_clock::now();

  context_.set_waypoints_client->async_send_request(
    request,
    [this](rclcpp::Client<navigation::srv::SetWaypoints>::SharedFuture future) {
      context_.pending_op_start = std::chrono::steady_clock::time_point{};
      auto response = future.get();
      if (response->success) {
        context_.remote_navigation_point_count = context_.map->points().size();
        context_.status_message = "Route sent to core";
        RCLCPP_INFO(logger_, "SetWaypoints confirmed.");
      } else {
        context_.status_message = "Core rejected route: " + response->message;
        RCLCPP_WARN(logger_, "SetWaypoints rejected: %s", response->message.c_str());
      }
    });
}

void NavigationRuntime::sendSetConfigRequest(const navigation::ControllerConfig & config)
{
  if (!serviceReady<rclcpp::Client<navigation::srv::SetControllerConfig>>(
      context_.set_config_client,
      context_,
      logger_,
      "SetControllerConfig"))
  {
    return;
  }

  auto request = std::make_shared<navigation::srv::SetControllerConfig::Request>();
  fillConfigRequest(*request, config);

  context_.status_message = "Sending params to core...";
  context_.pending_op_start = std::chrono::steady_clock::now();

  context_.set_config_client->async_send_request(
    request,
    [this](rclcpp::Client<navigation::srv::SetControllerConfig>::SharedFuture future) {
      context_.pending_op_start = std::chrono::steady_clock::time_point{};
      auto response = future.get();
      if (response->success) {
        context_.status_message = "Params sent to core";
        RCLCPP_INFO(logger_, "SetControllerConfig confirmed.");
      } else {
        context_.status_message = "Core rejected params: " + response->message;
        RCLCPP_WARN(logger_, "SetControllerConfig rejected: %s", response->message.c_str());
      }
    });
}

void NavigationRuntime::sendStartRequest(
  const std::vector<navigation::maps::MapPoint> & points,
  const navigation::ControllerConfig & config,
  const std::string & controller_name)
{
  if (!serviceReady<rclcpp::Client<navigation::srv::StartNavigation>>(
      context_.start_client,
      context_,
      logger_,
      "StartNavigation"))
  {
    return;
  }

  auto request = std::make_shared<navigation::srv::StartNavigation::Request>();
  for (const auto & point : points) {
    navigation::msg::MapPoint mp;
    mp.id = point.id;
    mp.x = point.x;
    mp.y = point.y;
    mp.fast = point.fast;
    request->points.push_back(mp);
  }
  request->race_logic = normalizeRaceLogic(context_.race_logic);
  fillStartConfigRequest(*request, config);
  request->controller_name = controller_name;

  context_.status_message = "Sending start to core...";
  context_.pending_op_start = std::chrono::steady_clock::now();

  context_.start_client->async_send_request(
    request,
    [this](rclcpp::Client<navigation::srv::StartNavigation>::SharedFuture future) {
      context_.pending_op_start = std::chrono::steady_clock::time_point{};
      auto response = future.get();
      if (response->success) {
        context_.remote_navigation_active = true;
        context_.remote_navigation_point_count = context_.map->points().size();
        context_.navigation_status = response->message;
        context_.status_message = "Navigation started";
        RCLCPP_INFO(logger_, "Core confirmed navigation start.");
      } else {
        context_.remote_navigation_active = false;
        context_.navigation_status = "Start rejected";
        context_.status_message = "Core rejected start: " + response->message;
        RCLCPP_WARN(logger_, "StartNavigation rejected: %s", response->message.c_str());
      }
    });
}

void NavigationRuntime::sendStopRequest(const std::string & reason)
{
  if (!serviceReady<rclcpp::Client<navigation::srv::StopNavigation>>(
      context_.stop_client,
      context_,
      logger_,
      "StopNavigation"))
  {
    return;
  }

  auto request = std::make_shared<navigation::srv::StopNavigation::Request>();
  request->reason = reason;

  context_.status_message = "Sending stop to core...";
  context_.pending_op_start = std::chrono::steady_clock::now();

  context_.stop_client->async_send_request(
    request,
    [this, reason](rclcpp::Client<navigation::srv::StopNavigation>::SharedFuture future) {
      context_.pending_op_start = std::chrono::steady_clock::time_point{};
      auto response = future.get();
      if (response->success) {
        context_.remote_navigation_active = false;
        context_.navigation_status = reason;
        context_.status_message = reason;
        RCLCPP_INFO(logger_, "Core confirmed stop: %s", reason.c_str());
      } else {
        context_.status_message = "Core rejected stop: " + response->message;
        RCLCPP_WARN(logger_, "StopNavigation rejected: %s", response->message.c_str());
      }
    });
}

void NavigationRuntime::updateNavigationController(
  bool has_state,
  const navigation::RobotNavigationState & state)
{
  if (context_.remote_control) {
    return;
  }

  if (!isNavigationActive()) {
    return;
  }

  if (!has_state) {
    publishZeroVelocity();
    context_.navigation_status = "Waiting for pose";
    return;
  }

  if (context_.race_logic == "mission") {
    if (context_.mission_paused) {
      publishZeroVelocity();
      sendArrivedToArmIfDue();
      return;
    }

    if (maybePauseForMissionTask(state)) {
      publishZeroVelocity();
      sendArrivedToArmIfDue();
      return;
    }
  }

  const auto command = context_.controller->update(state);
  if (publish_velocity_ != nullptr) {
    publish_velocity_(command);
  }

  const auto controller_status = context_.controller->status();
  context_.navigation_status = controller_status.message;
  if (controller_status.complete) {
    context_.status_message = "Navigation complete";
    RCLCPP_INFO(logger_, "Navigation route complete.");
  }
}

std::vector<navigation::maps::MapPoint> NavigationRuntime::controllerWaypointsForCurrentRace() const
{
  auto points = context_.map->points();
  if (context_.race_logic == "mission") {
    for (auto & point : points) {
      point.fast = false;
    }
  }
  return points;
}

void NavigationRuntime::resetMissionTasks()
{
  clearMissionPause();
  context_.mission_tasks.clear();
  if (context_.race_logic != "mission" || context_.map == nullptr) {
    return;
  }

  const auto & points = context_.map->points();
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!points[i].fast) {
      continue;
    }
    NavigationNodeContext::MissionTaskState task;
    task.point_index = i;
    task.point_id = points[i].id;
    context_.mission_tasks.push_back(task);
  }
}

void NavigationRuntime::clearMissionPause()
{
  context_.mission_paused = false;
  context_.mission_arrived_request_pending = false;
  context_.mission_current_task = 0;
  context_.mission_last_arrived_send = std::chrono::steady_clock::time_point{};
}

bool NavigationRuntime::shouldValidateFastMarkers() const
{
  return context_.race_logic != "mission";
}

bool NavigationRuntime::shouldResumeForEvent(const std::string & event) const
{
  return context_.mission_resume_event == event;
}

bool NavigationRuntime::maybePauseForMissionTask(const navigation::RobotNavigationState & state)
{
  if (context_.map == nullptr) {
    return false;
  }

  const auto & points = context_.map->points();
  for (std::size_t i = 0; i < context_.mission_tasks.size(); ++i) {
    auto & task = context_.mission_tasks[i];
    if (task.triggered || task.point_index >= points.size()) {
      continue;
    }

    const auto & point = points[task.point_index];
    const double distance = std::hypot(point.x - state.x, point.y - state.y);
    if (distance > context_.mission_task_radius) {
      continue;
    }

    task.triggered = true;
    context_.mission_paused = true;
    context_.mission_current_task = i;
    context_.mission_last_arrived_send = std::chrono::steady_clock::time_point{};
    context_.navigation_status = "Mission task paused at point " + std::to_string(task.point_id);
    context_.status_message = "Mission task arrived";
    RCLCPP_INFO(
      logger_,
      "Mission task point %d reached within %.3fm. Navigation paused.",
      task.point_id,
      context_.mission_task_radius);
    return true;
  }
  return false;
}

void NavigationRuntime::sendArrivedToArmIfDue()
{
  if (!context_.mission_paused || context_.mission_current_task >= context_.mission_tasks.size()) {
    return;
  }
  if (context_.mission_arrived_request_pending) {
    return;
  }

  auto & task = context_.mission_tasks[context_.mission_current_task];
  if (task.ack) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (context_.mission_last_arrived_send != std::chrono::steady_clock::time_point{}) {
    const auto elapsed = std::chrono::duration<double>(now - context_.mission_last_arrived_send).count();
    if (elapsed < context_.mission_arm_retry_period) {
      return;
    }
  }

  if (context_.arm_mission_client == nullptr || !context_.arm_mission_client->service_is_ready()) {
    context_.mission_last_arrived_send = now;
    context_.status_message = "Waiting for arm service";
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(
    logger_,
    "Sending mission arrived trigger to arm service '%s' for point %d.",
    context_.arm_mission_service.c_str(),
    task.point_id);
  context_.mission_arrived_request_pending = true;
  context_.mission_last_arrived_send = now;
  context_.arm_mission_client->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      context_.mission_arrived_request_pending = false;
      if (!context_.mission_paused || context_.mission_current_task >= context_.mission_tasks.size()) {
        return;
      }

      auto response = future.get();
      if (!response->success) {
        context_.status_message = "Arm rejected arrived: " + response->message;
        RCLCPP_WARN(logger_, "Arm mission arrived rejected: %s", response->message.c_str());
        return;
      }

      auto & current_task = context_.mission_tasks[context_.mission_current_task];
      current_task.ack = true;
      context_.status_message = "Arm ack received";
      RCLCPP_INFO(logger_, "Arm acknowledged mission arrived for point %d.", current_task.point_id);
      if (shouldResumeForEvent("ack")) {
        resumeMissionNavigation("Mission resumed after arm ack");
      }
    });
}

void NavigationRuntime::resumeMissionNavigation(const std::string & reason)
{
  context_.mission_paused = false;
  context_.mission_arrived_request_pending = false;
  context_.navigation_status = reason;
  context_.status_message = reason;
  RCLCPP_INFO(logger_, "%s", reason.c_str());
}

bool NavigationRuntime::handleArmEvent(const std::string & event, std::string * response_message)
{
  if (context_.race_logic != "mission") {
    if (response_message != nullptr) {
      *response_message = "received outside mission mode";
    }
    return true;
  }

  if (event != "grabbed" && event != "completed") {
    if (response_message != nullptr) {
      *response_message = "unknown event";
    }
    return false;
  }

  std::size_t event_task_index = context_.mission_current_task;
  if (!context_.mission_paused) {
    event_task_index = context_.mission_tasks.size();
    for (std::size_t i = 0; i < context_.mission_tasks.size(); ++i) {
      const auto & candidate = context_.mission_tasks[i];
      if (!candidate.triggered) {
        continue;
      }
      if ((event == "grabbed" && !candidate.grabbed) ||
        (event == "completed" && !candidate.completed))
      {
        event_task_index = i;
        break;
      }
    }
  }

  if (event_task_index >= context_.mission_tasks.size() ||
    !context_.mission_tasks[event_task_index].triggered)
  {
    if (response_message != nullptr) {
      *response_message = "received without active mission task";
    }
    return true;
  }

  auto & task = context_.mission_tasks[event_task_index];
  if (event == "grabbed") {
    task.grabbed = true;
  } else {
    task.completed = true;
  }

  if (response_message != nullptr) {
    *response_message = "received";
  }
  context_.status_message = "Arm event: " + event;
  RCLCPP_INFO(logger_, "Arm event '%s' received for mission point %d.", event.c_str(), task.point_id);
  if (context_.mission_paused && event_task_index == context_.mission_current_task &&
    shouldResumeForEvent(event))
  {
    resumeMissionNavigation("Mission resumed after arm " + event);
  }
  return true;
}

}  // namespace navigation::app
