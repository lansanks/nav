#include "app/navigation_map_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <sstream>
#include <memory>
#include <string>
#include <unordered_map>

#include "controller.hpp"
#include "interface.hpp"
#include "keyboards/navigation_keys.hpp"
#include "maps/point_store.hpp"
#include "maps/top_view_map.hpp"
#include "opencv2/highgui.hpp"
#include "params/navigation_params.hpp"

namespace navigation::app
{
namespace
{

double yawFromQuaternionWxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

std::unordered_map<std::string, std::string> parseStatusFields(const std::string & text)
{
  std::unordered_map<std::string, std::string> fields;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const auto sep = item.find('=');
    if (sep == std::string::npos) {
      continue;
    }
    fields[item.substr(0, sep)] = item.substr(sep + 1);
  }
  return fields;
}

std::size_t parseSize(const std::unordered_map<std::string, std::string> & fields, const std::string & key)
{
  const auto iter = fields.find(key);
  if (iter == fields.end()) {
    return 0;
  }
  try {
    return static_cast<std::size_t>(std::stoul(iter->second));
  } catch (const std::exception &) {
    return 0;
  }
}

bool parseBool(const std::unordered_map<std::string, std::string> & fields, const std::string & key)
{
  const auto iter = fields.find(key);
  return iter != fields.end() && (iter->second == "1" || iter->second == "true");
}

}  // namespace

NavigationMapNode::NavigationMapNode()
: Node("navigation_map"),
  runtime_(
    context_,
    get_logger(),
    [this](const geometry_msgs::msg::Twist & command) {
      publishVelocity(command);
    }),
  points_workflow_(context_, runtime_, get_logger()),
  ui_coordinator_(context_, runtime_, points_workflow_, *this, get_logger()),
  mouse_controller_(context_, ui_coordinator_, points_workflow_, get_logger())
{
  const auto config = navigation::params::declareRuntimeConfig(*this);
  context_.robot_name = config.robot_name;
  context_.points_file = config.points_file;
  context_.show_window = config.show_window;
  context_.cmd_vel_topic = config.cmd_vel_topic;
  context_.map_width_px = config.map_width_px;
  context_.map_height_px = config.map_height_px;
  context_.panel_collapsed = config.panel_collapsed;
  context_.radar_topic = declare_parameter<std::string>("radar_calibration_topic", "/Odometry");
  context_.controller_config = config.controller_config;
  context_.param_fields = navigation::params::makeControllerParamFields(context_.controller_config);
  context_.race_logic = config.race_logic;
  context_.mission_task_radius = config.mission_task_radius;
  context_.mission_resume_event = config.mission_resume_event;
  context_.arm_mission_service = config.arm_mission_service;
  context_.navigation_arm_event_service = config.navigation_arm_event_service;
  context_.mission_arm_retry_period = config.mission_arm_retry_period;
  const auto node_role = declare_parameter<std::string>("node_role", "standalone");
  context_.remote_control = node_role == "remote_ui";
  const auto state_topic = declare_parameter<std::string>("navigation_state_topic", "/navigation/state");
  const auto status_topic = declare_parameter<std::string>("navigation_status_topic", "/navigation/status");

  const auto scene_path = navigation::maps::resolveScenePath(context_.robot_name, config.scene);
  context_.current_map_file = scene_path;
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

  if (!context_.remote_control) {
    if (config.source == "radar") {
      context_.interface = navigation::createRadarInterface();
    } else {
      context_.interface = navigation::createSimulationInterface();
    }
    context_.interface->start(*this);
  }

  context_.window_name = config.window_name;
  if (context_.show_window) {
    cv::namedWindow(
      context_.window_name,
      cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_NORMAL);
    cv::setWindowProperty(context_.window_name, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_FREERATIO);
    cv::setMouseCallback(context_.window_name, &NavigationMapNode::onMouse, this);
    scroll_controller_ = std::make_unique<navigation::ui::WindowScrollController>(
      context_.window_name,
      [this](int delta, int x, int y) {
        mouse_controller_.handleWheelDelta(x, y, delta);
      });
    if (!scroll_controller_->installQtEventFilter()) {
      RCLCPP_WARN(get_logger(), "Qt application instance is unavailable; mouse wheel zoom is disabled.");
    }
  }

  if (context_.remote_control) {
    // Do NOT initialize last_heartbeat_time — let it stay at zero.
    // onTimer() checks for zero to know no heartbeat has ever arrived,
    // avoiding the time-source mismatch from subtracting different clock types.

    // Create service clients for communication with core
    context_.set_waypoints_client =
      create_client<navigation::srv::SetWaypoints>("/navigation/set_waypoints");
    context_.set_config_client =
      create_client<navigation::srv::SetControllerConfig>("/navigation/set_config");
    context_.start_client =
      create_client<navigation::srv::StartNavigation>("/navigation/start");
    context_.stop_client =
      create_client<navigation::srv::StopNavigation>("/navigation/stop");

    // Subscribe to state and status from core
    remote_state_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        handleRemoteState(msg);
      });
    remote_status_subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic,
      rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr msg) {
        handleRemoteStatus(msg);
      });

    // Subscribe to heartbeat from core for connection monitoring
    heartbeat_subscription_ = create_subscription<std_msgs::msg::String>(
      "/navigation/heartbeat",
      rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr /* msg */) {
        context_.last_heartbeat_time = now();
      });

    context_.navigation_status = "Waiting for core";
  } else {
    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(context_.cmd_vel_topic, rclcpp::QoS(10));
    context_.arm_mission_client =
      create_client<std_srvs::srv::Trigger>(context_.arm_mission_service);
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
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / config.update_rate_hz),
    [this]() {
      onTimer();
    });

  RCLCPP_INFO(
    get_logger(),
    "Loaded top-view map '%s' with %zu drawable geoms.",
    context_.current_map_file.c_str(),
    context_.map->geomCount());
  RCLCPP_INFO(get_logger(), "Navigation points file: %s", context_.points_file.c_str());
  if (context_.remote_control) {
    RCLCPP_INFO(
      get_logger(),
      "Remote UI mode. services on /navigation/{set_waypoints,set_config,start,stop}, "
      "state: %s, status: %s",
      state_topic.c_str(),
      status_topic.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Navigation command topic: %s", context_.cmd_vel_topic.c_str());
  }
}

void NavigationMapNode::onMouse(int event, int x, int y, int flags, void * userdata)
{
  auto * node = static_cast<NavigationMapNode *>(userdata);
  if (node == nullptr) {
    return;
  }

  node->mouse_controller_.handleMouseEvent(event, x, y, flags);
}

void NavigationMapNode::publishVelocity(const geometry_msgs::msg::Twist & command)
{
  if (context_.remote_control) {
    return;
  }

  if (cmd_vel_publisher_ != nullptr) {
    cmd_vel_publisher_->publish(command);
  }
}

void NavigationMapNode::handleRemoteState(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  navigation::RobotNavigationState next;
  next.valid = true;
  next.stamp = rclcpp::Time(msg->header.stamp);
  next.frame_id = msg->header.frame_id;
  next.source = "remote";
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
  context_.remote_latest_state = next;
}

void NavigationMapNode::handleRemoteStatus(const std_msgs::msg::String::SharedPtr msg)
{
  const auto fields = parseStatusFields(msg->data);
  context_.remote_navigation_active = parseBool(fields, "active");
  context_.remote_navigation_complete = parseBool(fields, "complete");
  context_.remote_navigation_target_index = parseSize(fields, "target");
  context_.remote_navigation_point_count = parseSize(fields, "points");

  const auto controller = fields.find("controller");
  if (controller != fields.end() && !controller->second.empty()) {
    context_.selected_controller_name = controller->second;
  }

  const auto race = fields.find("race");
  if (race != fields.end() && !race->second.empty()) {
    context_.race_logic = race->second == "mission" ? "mission" : "obstacle";
  }

  const auto status = fields.find("status");
  if (status != fields.end()) {
    context_.navigation_status = status->second;
  }
}

void NavigationMapNode::handleArmEvent(
  const std::shared_ptr<navigation::srv::StringCommand::Request> request,
  std::shared_ptr<navigation::srv::StringCommand::Response> response)
{
  std::string message;
  response->success = runtime_.handleArmEvent(request->message, &message);
  response->message = message.empty() ? "received" : message;
}

void NavigationMapNode::onTimer()
{
  if (scroll_controller_ != nullptr) {
    scroll_controller_->installGtkScrollController();
  }

  // Check heartbeat in remote_ui mode
  if (context_.remote_control) {
    const bool was_connected = context_.core_connected;

    if (context_.last_heartbeat_time.nanoseconds() == 0) {
      // No heartbeat received yet
      context_.core_connected = false;
    } else {
      const auto elapsed = (now() - context_.last_heartbeat_time).seconds();
      context_.core_connected = elapsed < NavigationNodeContext::HEARTBEAT_TIMEOUT_S;
    }

    // Update status_message immediately on connection state change
    if (!context_.core_connected) {
      context_.status_message = "Core DISCONNECTED";
    } else if (!was_connected && context_.core_connected) {
      context_.status_message.clear();
    }

    // Operation timeout: if a service request has been pending too long, show timeout
    if (context_.core_connected &&
        context_.pending_op_start != std::chrono::steady_clock::time_point{}) {
      const auto op_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - context_.pending_op_start).count();
      if (op_elapsed > NavigationNodeContext::OP_TIMEOUT_S) {
        context_.status_message = "Operation timed out - check core connection";
        context_.pending_op_start = std::chrono::steady_clock::time_point{};
      }
    }
  }

  navigation::RobotNavigationState state;
  bool has_state = false;
  if (context_.remote_control) {
    state = context_.remote_latest_state;
    has_state = state.valid;
  } else {
    has_state = context_.interface != nullptr && context_.interface->getState(state);
    runtime_.updateNavigationController(has_state, state);
  }
  const auto ui_state = ui_coordinator_.buildUiState();

  auto frame = context_.map->draw(has_state ? &state : nullptr, ui_state);
  if (!context_.show_window) {
    return;
  }

  applyFullscreenIfNeeded(frame.cols, frame.rows);
  cv::imshow(context_.window_name, frame);
  const int key = cv::waitKeyEx(1);
  if (ui_coordinator_.handleActiveInputKey(key)) {
    return;
  }

  const int ascii = navigation::keyboards::keyAscii(key);
  if (navigation::keyboards::isEscKey(key) || ascii == 'q' || ascii == 'Q') {
    rclcpp::shutdown();
  } else if (ascii == 'c' || ascii == 'C') {
    points_workflow_.clearPoints();
  }
}

void NavigationMapNode::applyFullscreenIfNeeded(int frame_width, int frame_height)
{
  const bool size_changed =
    frame_width != displayed_frame_width_ || frame_height != displayed_frame_height_;

  if (!context_.fullscreen_dirty && !size_changed) {
    return;
  }

  if (context_.fullscreen_dirty) {
    cv::setWindowProperty(
      context_.window_name,
      cv::WND_PROP_FULLSCREEN,
      context_.fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
    cv::setWindowProperty(context_.window_name, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_FREERATIO);
  }

  if (!context_.fullscreen && size_changed) {
    cv::setWindowProperty(context_.window_name, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_FREERATIO);
    cv::resizeWindow(context_.window_name, frame_width, frame_height);
  }
  context_.fullscreen_dirty = false;
  displayed_frame_width_ = frame_width;
  displayed_frame_height_ = frame_height;
}

}  // namespace navigation::app
