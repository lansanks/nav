#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "app/navigation_node_context.hpp"
#include "app/navigation_runtime.hpp"
#include "controller.hpp"
#include "interface.hpp"
#include "maps/navigation_map_helpers.hpp"
#include "maps/point_store.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navigation/srv/set_waypoints.hpp"
#include "navigation/srv/set_controller_config.hpp"
#include "navigation/srv/start_navigation.hpp"
#include "navigation/srv/stop_navigation.hpp"
#include "navigation/srv/string_command.hpp"
#include "params/navigation_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation::app
{
namespace
{

std::string sanitizeField(std::string value)
{
  std::replace(value.begin(), value.end(), ';', ' ');
  std::replace(value.begin(), value.end(), '=', ' ');
  return value;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.z = std::sin(yaw * 0.5);
  return q;
}

void applyConfigFromRequest(
  navigation::ControllerConfig & config,
  double waypoint_tolerance,
  double max_linear_speed,
  double max_angular_speed,
  double k_rho, double k_alpha, double k_beta,
  double fast_max_linear_speed, double fast_max_angular_speed,
  double fast_k_rho, double fast_k_alpha, double fast_k_beta)
{
  if (waypoint_tolerance > 0.0) { config.waypoint_tolerance = waypoint_tolerance; }
  if (max_linear_speed > 0.0) { config.max_linear_speed = max_linear_speed; }
  if (max_angular_speed > 0.0) { config.max_angular_speed = max_angular_speed; }
  config.k_rho = k_rho;
  config.k_alpha = k_alpha;
  config.k_beta = k_beta;
  if (fast_max_linear_speed > 0.0) { config.fast_max_linear_speed = fast_max_linear_speed; }
  if (fast_max_angular_speed > 0.0) { config.fast_max_angular_speed = fast_max_angular_speed; }
  config.fast_k_rho = fast_k_rho;
  config.fast_k_alpha = fast_k_alpha;
  config.fast_k_beta = fast_k_beta;
}

std::string normalizeRaceLogic(const std::string & race_logic)
{
  return race_logic == "mission" ? "mission" : "obstacle";
}

std::vector<navigation::maps::MapPoint> controllerPointsForRace(
  const std::vector<navigation::maps::MapPoint> & points,
  const std::string & race_logic)
{
  auto controller_points = points;
  if (race_logic == "mission") {
    for (auto & point : controller_points) {
      point.fast = false;
    }
  }
  return controller_points;
}

}  // namespace

class NavigationCoreNode final : public rclcpp::Node
{
public:
  NavigationCoreNode()
  : Node("navigation_core"),
    runtime_(
      context_,
      get_logger(),
      [this](const geometry_msgs::msg::Twist & command) {
        publishVelocity(command);
      })
  {
    const auto config = navigation::params::declareRuntimeConfig(*this);
    context_.robot_name = config.robot_name;
    context_.points_file = config.points_file;
    context_.show_window = false;
    context_.cmd_vel_topic = config.cmd_vel_topic;
    context_.controller_config = config.controller_config;
    context_.param_fields = navigation::params::makeControllerParamFields(context_.controller_config);
    context_.race_logic = config.race_logic;
    context_.mission_task_radius = config.mission_task_radius;
    context_.mission_resume_event = config.mission_resume_event;
    context_.arm_mission_service = config.arm_mission_service;
    context_.navigation_arm_event_service = config.navigation_arm_event_service;
    context_.mission_arm_retry_period = config.mission_arm_retry_period;

    context_.current_map_file = navigation::maps::resolveScenePath(context_.robot_name, config.scene);
    context_.map = std::make_unique<navigation::maps::TopViewMap>(
      config.map_width_px,
      config.map_height_px,
      config.map_padding_px);
    context_.map->load(context_.current_map_file);
    context_.map->setPoints(navigation::maps::loadPointsFile(context_.points_file));
    context_.controller_names = navigation::controllerNames();
    if (!context_.controller_names.empty()) {
      context_.selected_controller_name = context_.controller_names.front();
      context_.navigation_status = "Stopped";
    } else {
      context_.navigation_status = "No controllers";
    }

    if (config.source == "radar") {
      context_.interface = navigation::createRadarInterface();
    } else {
      context_.interface = navigation::createSimulationInterface();
    }
    context_.interface->start(*this);

    const auto status_topic = declare_parameter<std::string>("navigation_status_topic", "/navigation/status");
    const auto state_topic = declare_parameter<std::string>("navigation_state_topic", "/navigation/state");

    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(context_.cmd_vel_topic, rclcpp::QoS(10));

    // Service servers (replacing topic-based command subscription)
    set_waypoints_service_ = create_service<navigation::srv::SetWaypoints>(
      "/navigation/set_waypoints",
      [this](
        const std::shared_ptr<navigation::srv::SetWaypoints::Request> request,
        std::shared_ptr<navigation::srv::SetWaypoints::Response> response) {
        handleSetWaypoints(request, response);
      });

    set_config_service_ = create_service<navigation::srv::SetControllerConfig>(
      "/navigation/set_config",
      [this](
        const std::shared_ptr<navigation::srv::SetControllerConfig::Request> request,
        std::shared_ptr<navigation::srv::SetControllerConfig::Response> response) {
        handleSetConfig(request, response);
      });

    start_service_ = create_service<navigation::srv::StartNavigation>(
      "/navigation/start",
      [this](
        const std::shared_ptr<navigation::srv::StartNavigation::Request> request,
        std::shared_ptr<navigation::srv::StartNavigation::Response> response) {
        handleStartNavigation(request, response);
      });

    stop_service_ = create_service<navigation::srv::StopNavigation>(
      "/navigation/stop",
      [this](
        const std::shared_ptr<navigation::srv::StopNavigation::Request> request,
        std::shared_ptr<navigation::srv::StopNavigation::Response> response) {
        handleStopNavigation(request, response);
      });

    arm_event_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    arm_event_service_ = create_service<navigation::srv::StringCommand>(
      context_.navigation_arm_event_service,
      [this](
        const std::shared_ptr<navigation::srv::StringCommand::Request> request,
        std::shared_ptr<navigation::srv::StringCommand::Response> response) {
        handleArmEvent(request, response);
      },
      rmw_qos_profile_services_default,
      arm_event_callback_group_);

    context_.arm_mission_client =
      create_client<std_srvs::srv::Trigger>(context_.arm_mission_service);

    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(10));
    state_publisher_ = create_publisher<nav_msgs::msg::Odometry>(state_topic, rclcpp::SensorDataQoS());

    // Heartbeat publisher: sends a pulse every second so remote UI can detect disconnection
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>("/navigation/heartbeat", rclcpp::QoS(10));
    heartbeat_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        std_msgs::msg::String msg;
        msg.data = "beat";
        heartbeat_publisher_->publish(msg);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / config.update_rate_hz),
      [this]() {
        onTimer();
      });

    RCLCPP_INFO(
      get_logger(),
      "Navigation core ready. services: /navigation/{set_waypoints,set_config,start,stop}, "
      "status: %s, state: %s, cmd_vel: %s",
      status_topic.c_str(),
      state_topic.c_str(),
      context_.cmd_vel_topic.c_str());
  }

private:
  void publishVelocity(const geometry_msgs::msg::Twist & command)
  {
    if (cmd_vel_publisher_ != nullptr) {
      cmd_vel_publisher_->publish(command);
    }
  }

  void onTimer()
  {
    navigation::RobotNavigationState state;
    const bool has_state = context_.interface != nullptr && context_.interface->getState(state);
    runtime_.updateNavigationController(has_state, state);
    if (has_state) {
      publishState(state);
    }
    publishStatus();
  }

  void publishState(const navigation::RobotNavigationState & state)
  {
    nav_msgs::msg::Odometry msg;
    const auto stamp = state.stamp.nanoseconds() == 0 ? now() : state.stamp;
    const auto stamp_ns = stamp.nanoseconds();
    msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
    msg.header.frame_id = state.frame_id.empty() ? "map" : state.frame_id;
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = state.x;
    msg.pose.pose.position.y = state.y;
    msg.pose.pose.position.z = state.z;
    msg.pose.pose.orientation = quaternionFromYaw(state.yaw);
    msg.twist.twist.linear.x = state.linear_x;
    msg.twist.twist.linear.y = state.linear_y;
    msg.twist.twist.linear.z = state.linear_z;
    msg.twist.twist.angular.z = state.angular_z;
    state_publisher_->publish(msg);
  }

  void publishStatus()
  {
    bool active = false;
    bool complete = false;
    std::size_t target_index = 0;
    std::size_t point_count = context_.map != nullptr ? context_.map->points().size() : 0;
    std::string status = context_.navigation_status;
    if (context_.controller != nullptr) {
      const auto controller_status = context_.controller->status();
      active = controller_status.active;
      complete = controller_status.complete;
      target_index = controller_status.target_index;
      point_count = controller_status.point_count;
      if (!controller_status.message.empty()) {
        status = controller_status.message;
      }
    }
    if (context_.mission_paused && !context_.navigation_status.empty()) {
      status = context_.navigation_status;
    }

    std_msgs::msg::String msg;
    msg.data = "active=" + std::string(active ? "1" : "0") +
      ";complete=" + std::string(complete ? "1" : "0") +
      ";target=" + std::to_string(target_index) +
      ";points=" + std::to_string(point_count) +
      ";controller=" + sanitizeField(context_.selected_controller_name) +
      ";race=" + sanitizeField(context_.race_logic) +
      ";status=" + sanitizeField(status);
    status_publisher_->publish(msg);
  }

  // === Service handlers ===

  void handleSetWaypoints(
    const std::shared_ptr<navigation::srv::SetWaypoints::Request> request,
    std::shared_ptr<navigation::srv::SetWaypoints::Response> response)
  {
    std::vector<navigation::maps::MapPoint> points;
    for (const auto & mp : request->points) {
      navigation::maps::MapPoint point;
      point.id = mp.id;
      point.x = mp.x;
      point.y = mp.y;
      point.fast = mp.fast;
      points.push_back(point);
    }

    runtime_.stopNavigationForRouteChange();
    context_.race_logic = normalizeRaceLogic(request->race_logic);
    context_.map->setPoints(points);
    runtime_.syncControllerWaypoints();
    context_.navigation_status = "Route updated";

    response->success = true;
    response->message = "Route updated: " + std::to_string(points.size()) + " points";

    RCLCPP_INFO(get_logger(), "Service set_waypoints: %zu points.", points.size());
    publishStatus();
  }

  void handleSetConfig(
    const std::shared_ptr<navigation::srv::SetControllerConfig::Request> request,
    std::shared_ptr<navigation::srv::SetControllerConfig::Response> response)
  {
    applyConfigFromRequest(
      context_.controller_config,
      request->waypoint_tolerance,
      request->max_linear_speed,
      request->max_angular_speed,
      request->k_rho, request->k_alpha, request->k_beta,
      request->fast_max_linear_speed,
      request->fast_max_angular_speed,
      request->fast_k_rho, request->fast_k_alpha, request->fast_k_beta);

    runtime_.applyControllerConfig();
    context_.navigation_status = "Params updated";

    response->success = true;
    response->message = "Params updated";

    RCLCPP_INFO(get_logger(), "Service set_config: navigation params updated.");
    publishStatus();
  }

  void handleStartNavigation(
    const std::shared_ptr<navigation::srv::StartNavigation::Request> request,
    std::shared_ptr<navigation::srv::StartNavigation::Response> response)
  {
    // Stop any running navigation first
    if (runtime_.isNavigationActive()) {
      runtime_.stopNavigation("Navigation restarted");
    }

    // Set points
    std::vector<navigation::maps::MapPoint> points;
    for (const auto & mp : request->points) {
      navigation::maps::MapPoint point;
      point.id = mp.id;
      point.x = mp.x;
      point.y = mp.y;
      point.fast = mp.fast;
      points.push_back(point);
    }
    context_.map->setPoints(points);
    context_.race_logic = normalizeRaceLogic(request->race_logic);

    // Set config
    applyConfigFromRequest(
      context_.controller_config,
      request->waypoint_tolerance,
      request->max_linear_speed,
      request->max_angular_speed,
      request->k_rho, request->k_alpha, request->k_beta,
      request->fast_max_linear_speed,
      request->fast_max_angular_speed,
      request->fast_k_rho, request->fast_k_alpha, request->fast_k_beta);

    runtime_.applyControllerConfig();
    runtime_.syncControllerWaypoints();

    // Validate fast markers
    std::string marker_error;
    if (context_.race_logic == "obstacle" &&
      !navigation::maps::validateFastMarkers(context_.map->points(), &marker_error))
    {
      response->success = false;
      response->message = marker_error;
      RCLCPP_WARN(get_logger(), "Service start_navigation rejected: %s", marker_error.c_str());
      publishStatus();
      return;
    }

    // Start navigation
    const auto & controller_name = request->controller_name;
    context_.selected_controller_name = controller_name;

    auto next_controller = navigation::createController(controller_name, context_.controller_config);
    if (next_controller == nullptr) {
      response->success = false;
      response->message = "Controller unavailable: " + controller_name;
      RCLCPP_WARN(get_logger(), "Service start_navigation rejected: unknown controller '%s'.", controller_name.c_str());
      publishStatus();
      return;
    }

    next_controller->setWaypoints(controllerPointsForRace(context_.map->points(), context_.race_logic));

    std::string error_message;
    if (!next_controller->start(&error_message)) {
      context_.controller = std::move(next_controller);
      response->success = false;
      response->message = error_message.empty() ? "Navigation start failed" : error_message;
      context_.navigation_status = response->message;
      RCLCPP_WARN(get_logger(), "Service start_navigation failed: %s", response->message.c_str());
      publishStatus();
      return;
    }

    context_.controller = std::move(next_controller);
    context_.navigation_status = context_.controller->status().message;

    response->success = true;
    response->message = "Navigation started: " + controller_name;

    RCLCPP_INFO(get_logger(), "Service start_navigation: %zu points, controller '%s'.", points.size(), controller_name.c_str());
    publishStatus();
  }

  void handleStopNavigation(
    const std::shared_ptr<navigation::srv::StopNavigation::Request> request,
    std::shared_ptr<navigation::srv::StopNavigation::Response> response)
  {
    const auto reason = request->reason.empty() ? "Navigation stopped" : request->reason;
    runtime_.stopNavigation(reason);

    response->success = true;
    response->message = reason;

    RCLCPP_INFO(get_logger(), "Service stop_navigation: %s", reason.c_str());
    publishStatus();
  }

  void handleArmEvent(
    const std::shared_ptr<navigation::srv::StringCommand::Request> request,
    std::shared_ptr<navigation::srv::StringCommand::Response> response)
  {
    std::string message;
    response->success = runtime_.handleArmEvent(request->message, &message);
    response->message = message.empty() ? "received" : message;
    publishStatus();
  }

  NavigationNodeContext context_;
  NavigationRuntime runtime_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<navigation::srv::SetWaypoints>::SharedPtr set_waypoints_service_;
  rclcpp::Service<navigation::srv::SetControllerConfig>::SharedPtr set_config_service_;
  rclcpp::Service<navigation::srv::StartNavigation>::SharedPtr start_service_;
  rclcpp::Service<navigation::srv::StopNavigation>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr arm_event_callback_group_;
  rclcpp::Service<navigation::srv::StringCommand>::SharedPtr arm_event_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation::app

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<navigation::app::NavigationCoreNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_core"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
