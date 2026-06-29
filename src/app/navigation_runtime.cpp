#include "app/navigation_runtime.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "controller.hpp"
#include "maps/navigation_map_helpers.hpp"
#include "navigation/srv/set_waypoints.hpp"
#include "navigation/srv/set_controller_config.hpp"
#include "navigation/srv/start_navigation.hpp"
#include "navigation/srv/stop_navigation.hpp"
#include "navigation/srv/string_command.hpp"

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

std::uint8_t taskTypeForMissionPoint(const navigation::maps::MapPoint & point, std::size_t task_index)
{
  if (point.task_type == navigation::maps::kTaskTypePickup ||
    point.task_type == navigation::maps::kTaskTypePlace)
  {
    return point.task_type;
  }

  if (point.fast) {
    return task_index % 2 == 0 ? navigation::maps::kTaskTypePickup : navigation::maps::kTaskTypePlace;
  }

  return navigation::maps::kTaskTypeNone;
}

const char * taskActionText(std::uint8_t task_type)
{
  if (task_type == navigation::maps::kTaskTypePlace) {
    return "place";
  }
  return "pickup";
}

bool isEventValidForTask(
  const NavigationNodeContext::MissionTaskState & task,
  const std::string & event)
{
  if (event == "ack" || event == "completed") {
    return true;
  }
  if (task.task_type == navigation::maps::kTaskTypePlace) {
    return event == "placed";
  }
  return event == "grabbed";
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
  PublishVelocity publish_velocity,
  PublishEventCommand publish_event_command,
  PublishPolicyConfig publish_policy_config)
: context_(context),
  logger_(logger),
  publish_velocity_(std::move(publish_velocity)),
  publish_event_command_(std::move(publish_event_command)),
  publish_policy_config_(std::move(publish_policy_config))
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
  resetNavigationEventState();
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
  resetNavigationEventState();
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

void NavigationRuntime::applyRadarCalibrationFile(const std::string & path)
{
  if (context_.remote_control) {
    sendSetRadarCalibrationRequest(path);
    return;
  }

  if (context_.interface == nullptr) {
    context_.status_message = "Radar calibration interface unavailable";
    RCLCPP_WARN(logger_, "%s", context_.status_message.c_str());
    return;
  }

  std::string message;
  if (!context_.interface->setRadarCalibrationFile(path, &message)) {
    context_.status_message = "Calibration apply failed";
    RCLCPP_WARN(logger_, "Failed to apply radar calibration: %s", message.c_str());
    return;
  }

  context_.status_message = "Calibration applied";
  RCLCPP_INFO(logger_, "Applied radar calibration: %s", message.c_str());
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
    mp.task_type = point.task_type;
    mp.event_label = point.event_label;
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
    mp.task_type = point.task_type;
    mp.event_label = point.event_label;
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

void NavigationRuntime::sendSetRadarCalibrationRequest(const std::string & path)
{
  if (!serviceReady<rclcpp::Client<navigation::srv::StringCommand>>(
      context_.set_radar_calibration_client,
      context_,
      logger_,
      "SetRadarCalibration"))
  {
    return;
  }

  auto request = std::make_shared<navigation::srv::StringCommand::Request>();
  request->message = path;

  context_.status_message = "Sending calibration to core...";
  context_.pending_op_start = std::chrono::steady_clock::now();

  context_.set_radar_calibration_client->async_send_request(
    request,
    [this](rclcpp::Client<navigation::srv::StringCommand>::SharedFuture future) {
      context_.pending_op_start = std::chrono::steady_clock::time_point{};
      auto response = future.get();
      if (response->success) {
        context_.status_message = "Calibration applied to core";
        RCLCPP_INFO(logger_, "SetRadarCalibration confirmed: %s", response->message.c_str());
      } else {
        context_.status_message = "Core rejected calibration: " + response->message;
        RCLCPP_WARN(logger_, "SetRadarCalibration rejected: %s", response->message.c_str());
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

  if (maybeHoldForNavigationEvent()) {
    publishZeroVelocity();
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

  const auto before_status = context_.controller->status();
  const auto command = context_.controller->update(state);
  const auto after_status = context_.controller->status();
  if (handleArrivedNavigationEvents(before_status.target_index, after_status.target_index)) {
    publishZeroVelocity();
    return;
  }
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
  std::size_t task_index = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto task_type = taskTypeForMissionPoint(points[i], task_index);
    if (task_type == navigation::maps::kTaskTypeNone) {
      continue;
    }
    NavigationNodeContext::MissionTaskState task;
    task.point_index = i;
    task.point_id = points[i].id;
    task.task_type = task_type;
    context_.mission_tasks.push_back(task);
    ++task_index;
  }
}

void NavigationRuntime::clearMissionPause()
{
  context_.mission_paused = false;
  context_.mission_arrived_request_pending = false;
  context_.mission_current_task = 0;
  context_.mission_last_arrived_send = std::chrono::steady_clock::time_point{};
}

void NavigationRuntime::resetNavigationEventState()
{
  const std::size_t point_count = context_.map != nullptr ? context_.map->points().size() : 0;
  context_.navigation_event_triggered.assign(point_count, false);
  context_.navigation_event_wait_active = false;
  context_.navigation_event_wait_until = std::chrono::steady_clock::time_point{};
}

bool NavigationRuntime::maybeHoldForNavigationEvent()
{
  if (!context_.navigation_event_wait_active) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < context_.navigation_event_wait_until) {
    return true;
  }

  context_.navigation_event_wait_active = false;
  context_.navigation_event_wait_until = std::chrono::steady_clock::time_point{};
  context_.status_message = "Navigation event wait complete";
  return false;
}

bool NavigationRuntime::handleArrivedNavigationEvents(std::size_t begin_index, std::size_t end_index)
{
  if (context_.map == nullptr || end_index <= begin_index) {
    return false;
  }

  const auto & points = context_.map->points();
  const auto clamped_end = std::min(end_index, points.size());
  for (std::size_t i = begin_index; i < clamped_end; ++i) {
    if (triggerNavigationEvent(i, points[i])) {
      return true;
    }
  }
  return false;
}

bool NavigationRuntime::triggerNavigationEvent(
  std::size_t point_index,
  const navigation::maps::MapPoint & point)
{
  if (point.event_label.empty()) {
    return false;
  }

  if (context_.navigation_event_triggered.size() < context_.map->points().size()) {
    context_.navigation_event_triggered.resize(context_.map->points().size(), false);
  }
  if (point_index < context_.navigation_event_triggered.size() &&
    context_.navigation_event_triggered[point_index])
  {
    return false;
  }

  if (point_index < context_.navigation_event_triggered.size()) {
    context_.navigation_event_triggered[point_index] = true;
  }

  std::string normalized = point.event_label;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });

  std::string command;
  std::string policy_config;
  std::string event_name;
  if (normalized == "stand") {
    command = "1";
    event_name = "Stand";
  } else if (normalized == "bridge") {
    command = "2";
    event_name = "Bridge";
  } else if (normalized == "low") {
    command = "3";
    event_name = "Low-bar";
  } else if (normalized.size() > 1 && normalized.front() == '_') {
    policy_config = "himloco" + normalized;
    event_name = "Policy " + policy_config;
  } else {
    context_.status_message = "Navigation event ignored: " + point.event_label;
    return false;
  }

  if (!command.empty() && publish_event_command_ != nullptr) {
    publish_event_command_(command);
  }
  if (!policy_config.empty() && publish_policy_config_ != nullptr) {
    publish_policy_config_(policy_config);
  }

  const auto wait_seconds = std::max(0.0, context_.navigation_event_wait_seconds);
  context_.navigation_event_wait_active = wait_seconds > 0.0;
  context_.navigation_event_wait_until =
    std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(wait_seconds));
  context_.navigation_status = event_name + " event triggered at point " + std::to_string(point.id);
  context_.status_message = event_name + " command sent";
  RCLCPP_INFO(
    logger_,
    "Navigation event '%s' reached at point %d. Published rl debug key '%s', policy config '%s' and waiting %.3fs.",
    normalized.c_str(),
    point.id,
    command.c_str(),
    policy_config.c_str(),
    wait_seconds);
  return true;
}

bool NavigationRuntime::shouldValidateFastMarkers() const
{
  return context_.race_logic != "mission";
}

bool NavigationRuntime::shouldResumeForEvent(
  const NavigationNodeContext::MissionTaskState & task,
  const std::string & event) const
{
  if (task.task_type == navigation::maps::kTaskTypePlace) {
    return context_.mission_place_resume_event == event;
  }
  return context_.mission_pickup_resume_event == event;
}

bool NavigationRuntime::maybePauseForMissionTask(const navigation::RobotNavigationState & state)
{
  if (context_.map == nullptr) {
    return false;
  }

  const auto & points = context_.map->points();
  std::size_t next_task_index = context_.mission_tasks.size();
  for (std::size_t i = 0; i < context_.mission_tasks.size(); ++i) {
    if (!context_.mission_tasks[i].triggered) {
      next_task_index = i;
      break;
    }
  }

  if (next_task_index >= context_.mission_tasks.size()) {
    return false;
  }

  auto & task = context_.mission_tasks[next_task_index];
  if (task.point_index >= points.size()) {
    return false;
  }

  const auto & point = points[task.point_index];
  const double distance = std::hypot(point.x - state.x, point.y - state.y);
  if (distance > context_.mission_task_radius) {
    return false;
  }

  task.triggered = true;
  context_.mission_paused = true;
  context_.mission_current_task = next_task_index;
  context_.mission_last_arrived_send = std::chrono::steady_clock::time_point{};
  context_.navigation_status = std::string("Mission ") + taskActionText(task.task_type) +
    " paused at point " + std::to_string(task.point_id);
  context_.status_message = std::string("Mission ") + taskActionText(task.task_type) + " arrived";
  RCLCPP_INFO(
    logger_,
    "Mission %s point %d reached within %.3fm. Navigation paused.",
    taskActionText(task.task_type),
    task.point_id,
    context_.mission_task_radius);
  return true;
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

  const auto & point = context_.map->points()[task.point_index];
  auto request = std::make_shared<navigation::srv::MissionCommand::Request>();
  request->task_index = static_cast<std::uint32_t>(context_.mission_current_task);
  request->point_id = task.point_id;
  request->action = taskActionText(task.task_type);
  request->x = point.x;
  request->y = point.y;
  RCLCPP_INFO(
    logger_,
    "Sending mission %s command to arm service '%s' for point %d.",
    request->action.c_str(),
    context_.arm_mission_service.c_str(),
    task.point_id);
  context_.mission_arrived_request_pending = true;
  context_.mission_last_arrived_send = now;
  context_.arm_mission_client->async_send_request(
    request,
    [this](rclcpp::Client<navigation::srv::MissionCommand>::SharedFuture future) {
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
      RCLCPP_INFO(
        logger_,
        "Arm acknowledged mission %s for point %d.",
        taskActionText(current_task.task_type),
        current_task.point_id);
      if (shouldResumeForEvent(current_task, "ack")) {
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

  if (event != "grabbed" && event != "placed" && event != "completed") {
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
      if ((event == "grabbed" && candidate.task_type == navigation::maps::kTaskTypePickup &&
        !candidate.grabbed) ||
        (event == "placed" && candidate.task_type == navigation::maps::kTaskTypePlace &&
        !candidate.placed) ||
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
  if (!isEventValidForTask(task, event)) {
    if (response_message != nullptr) {
      *response_message = std::string("unexpected event for ") + taskActionText(task.task_type) + " task";
    }
    return false;
  }
  if (event == "grabbed") {
    task.grabbed = true;
  } else if (event == "placed") {
    task.placed = true;
  } else {
    task.completed = true;
  }

  if (response_message != nullptr) {
    *response_message = "received";
  }
  context_.status_message = "Arm event: " + event;
  RCLCPP_INFO(logger_, "Arm event '%s' received for mission point %d.", event.c_str(), task.point_id);
  if (context_.mission_paused && event_task_index == context_.mission_current_task &&
    shouldResumeForEvent(task, event))
  {
    resumeMissionNavigation("Mission resumed after arm " + event);
  }
  return true;
}

}  // namespace navigation::app
