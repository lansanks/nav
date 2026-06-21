#include "app/navigation_map_node.hpp"

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
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

std::string trimCopy(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string unquoteScalar(std::string value)
{
  value = trimCopy(value);
  if (value.size() < 2 || value.front() != '"' || value.back() != '"') {
    return value;
  }

  std::string output;
  output.reserve(value.size() - 2);
  bool escaping = false;
  for (std::size_t i = 1; i + 1 < value.size(); ++i) {
    const char ch = value[i];
    if (escaping) {
      output.push_back(ch);
      escaping = false;
    } else if (ch == '\\') {
      escaping = true;
    } else {
      output.push_back(ch);
    }
  }
  return output;
}

std::string quoteScalar(const std::string & value)
{
  std::string output = "\"";
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') {
      output.push_back('\\');
    }
    output.push_back(ch);
  }
  output.push_back('"');
  return output;
}

std::filesystem::path defaultUiStateFilePath(const std::string & points_file)
{
  const auto points_path = std::filesystem::path(points_file);
  const auto points_dir = points_path.parent_path();
  const auto config_dir = points_dir.empty() ? std::filesystem::path("config") : points_dir.parent_path();
  return config_dir / "ui_state.yaml";
}

std::unordered_map<std::string, std::string> loadScalarFile(const std::string & path)
{
  std::unordered_map<std::string, std::string> fields;
  std::ifstream input(path);
  if (!input.is_open()) {
    return fields;
  }

  std::string line;
  while (std::getline(input, line)) {
    auto text = trimCopy(line);
    if (text.empty() || text[0] == '#') {
      continue;
    }
    const auto comment = text.find(" #");
    if (comment != std::string::npos) {
      text = trimCopy(text.substr(0, comment));
    }
    const auto sep = text.find(':');
    if (sep == std::string::npos) {
      continue;
    }
    const auto key = trimCopy(text.substr(0, sep));
    const auto value = unquoteScalar(text.substr(sep + 1));
    if (!key.empty()) {
      fields[key] = value;
    }
  }
  return fields;
}

bool parseBoolScalar(const std::string & text, bool & value)
{
  auto normalized = trimCopy(text);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
    value = true;
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
    value = false;
    return true;
  }
  return false;
}

bool parseDoubleScalar(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stod(trimCopy(text), &consumed);
    if (consumed != trimCopy(text).size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

navigation::ui::MapPlanDisplayMode parsePlanDisplayMode(
  const std::string & text,
  navigation::ui::MapPlanDisplayMode fallback)
{
  auto normalized = trimCopy(text);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (normalized == "full") {
    return navigation::ui::MapPlanDisplayMode::Full;
  }
  if (normalized == "order_only" || normalized == "orderonly" || normalized == "order") {
    return navigation::ui::MapPlanDisplayMode::OrderOnly;
  }
  if (normalized == "hidden" || normalized == "off") {
    return navigation::ui::MapPlanDisplayMode::Hidden;
  }
  return fallback;
}

std::string planDisplayModeText(navigation::ui::MapPlanDisplayMode mode)
{
  using navigation::ui::MapPlanDisplayMode;
  if (mode == MapPlanDisplayMode::OrderOnly) {
    return "order_only";
  }
  if (mode == MapPlanDisplayMode::Hidden) {
    return "hidden";
  }
  return "full";
}

void applyStringField(
  const std::unordered_map<std::string, std::string> & fields,
  const std::string & key,
  std::string & value)
{
  const auto iter = fields.find(key);
  if (iter != fields.end()) {
    value = iter->second;
  }
}

void applyBoolField(
  const std::unordered_map<std::string, std::string> & fields,
  const std::string & key,
  bool & value)
{
  const auto iter = fields.find(key);
  if (iter == fields.end()) {
    return;
  }
  bool parsed = false;
  if (parseBoolScalar(iter->second, parsed)) {
    value = parsed;
  }
}

std::string serializePersistentUiState(const NavigationNodeContext & context)
{
  std::ostringstream output;
  output << "# Auto-generated by navigation UI. Runtime navigation active state is intentionally not saved.\n";
  output << "version: 1\n";
  output << "points_file: " << quoteScalar(context.points_file) << "\n";
  output << "current_map_file: " << quoteScalar(context.current_map_file) << "\n";
  output << "selected_controller_name: " << quoteScalar(context.selected_controller_name) << "\n";
  output << "race_logic: " << quoteScalar(context.race_logic) << "\n";
  output << "panel_collapsed: " << (context.panel_collapsed ? "true" : "false") << "\n";
  output << "light_theme: " << (context.light_theme ? "true" : "false") << "\n";
  output << "fullscreen: " << (context.fullscreen ? "true" : "false") << "\n";
  output << "mission_plan_display_mode: " <<
    quoteScalar(planDisplayModeText(context.mission_plan_display_mode)) << "\n";
  output << "radar_data_file: " << quoteScalar(context.radar_data_file) << "\n";
  output << "radar_points_file: " << quoteScalar(context.radar_points_file) << "\n";
  output << "mission_slot_categories: " << quoteScalar(context.mission_slot_categories_text) << "\n";
  output << "mission_high_score_category: " << quoteScalar(context.mission_high_score_category_text) << "\n";
  output << "mission_high_score_priority: " << quoteScalar(context.mission_high_score_priority_text) << "\n";
  output << "mission_cost_budget: " << quoteScalar(context.mission_cost_budget_text) << "\n";
  output << "mission_alpha: " << quoteScalar(context.mission_alpha_text) << "\n";
  output << "mission_beta: " << quoteScalar(context.mission_beta_text) << "\n";
  output << "mission_eta: " << quoteScalar(context.mission_eta_text) << "\n";
  output << "mission_g_pick_place: " << quoteScalar(context.mission_g_pick_place_text) << "\n";
  output << "mission_storage_near_distance: " << quoteScalar(context.mission_storage_near_distance_text) << "\n";
  output << "mission_storage_far_distance: " << quoteScalar(context.mission_storage_far_distance_text) << "\n";
  output << "mission_return_near_distance: " << quoteScalar(context.mission_return_near_distance_text) << "\n";
  output << "mission_return_far_distance: " << quoteScalar(context.mission_return_far_distance_text) << "\n";
  output << std::fixed << std::setprecision(6);
  for (const auto & field : context.param_fields) {
    if (field.value != nullptr) {
      output << "controller_" << field.name << ": " << *field.value << "\n";
    }
  }
  return output.str();
}

bool writeTextFile(const std::string & path, const std::string & text, std::string * error_message)
{
  const std::filesystem::path file_path(path);
  const auto parent = file_path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      if (error_message != nullptr) {
        *error_message = "failed to create directory '" + parent.string() + "': " + error.message();
      }
      return false;
    }
  }

  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) {
    if (error_message != nullptr) {
      *error_message = "failed to open UI state file for writing: " + path;
    }
    return false;
  }
  output << text;
  return true;
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
  context_.ui_state_file = defaultUiStateFilePath(config.points_file).string();
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
  context_.current_map_file = navigation::maps::resolveScenePath(context_.robot_name, config.scene);
  loadPersistentUiState();
  const auto node_role = declare_parameter<std::string>("node_role", "standalone");
  context_.remote_control = node_role == "remote_ui";
  const auto state_topic = declare_parameter<std::string>("navigation_state_topic", "/navigation/state");
  const auto status_topic = declare_parameter<std::string>("navigation_status_topic", "/navigation/status");

  context_.map = std::make_unique<navigation::maps::TopViewMap>(
    config.map_width_px,
    config.map_height_px,
    config.map_padding_px);
  try {
    context_.map->load(context_.current_map_file);
  } catch (const std::exception & error) {
    const auto fallback_scene = navigation::maps::resolveScenePath(context_.robot_name, config.scene);
    RCLCPP_WARN(
      get_logger(),
      "Failed to load persisted map '%s': %s. Falling back to '%s'.",
      context_.current_map_file.c_str(),
      error.what(),
      fallback_scene.c_str());
    context_.current_map_file = fallback_scene;
    context_.map->load(context_.current_map_file);
  }
  context_.map->setPoints(navigation::maps::loadPointsFile(context_.points_file));
  context_.controller_names = navigation::controllerNames();
  if (!context_.controller_names.empty()) {
    if (context_.selected_controller_name.empty() ||
      std::find(
        context_.controller_names.begin(),
        context_.controller_names.end(),
        context_.selected_controller_name) == context_.controller_names.end())
    {
      context_.selected_controller_name = context_.controller_names.front();
    }
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

    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      context_.cmd_vel_topic,
      rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        recordCommandVelocity(*msg);
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
      "state: %s, status: %s, cmd_vel: %s",
      state_topic.c_str(),
      status_topic.c_str(),
      context_.cmd_vel_topic.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Navigation command topic: %s", context_.cmd_vel_topic.c_str());
  }
  persisted_ui_state_snapshot_ = serializePersistentUiState(context_);
}

NavigationMapNode::~NavigationMapNode()
{
  savePersistentUiStateIfChanged(true);
}

void NavigationMapNode::loadPersistentUiState()
{
  const auto fields = loadScalarFile(context_.ui_state_file);
  if (fields.empty()) {
    return;
  }

  applyStringField(fields, "points_file", context_.points_file);
  applyStringField(fields, "current_map_file", context_.current_map_file);
  applyStringField(fields, "selected_controller_name", context_.selected_controller_name);
  applyStringField(fields, "radar_data_file", context_.radar_data_file);
  applyStringField(fields, "radar_points_file", context_.radar_points_file);
  context_.radar_save_file_confirmed = !context_.radar_data_file.empty();

  std::string race_logic = context_.race_logic;
  applyStringField(fields, "race_logic", race_logic);
  context_.race_logic = race_logic == "mission" ? "mission" : "obstacle";

  applyBoolField(fields, "panel_collapsed", context_.panel_collapsed);
  applyBoolField(fields, "light_theme", context_.light_theme);
  applyBoolField(fields, "fullscreen", context_.fullscreen);
  context_.fullscreen_dirty = context_.fullscreen;

  const auto plan_mode = fields.find("mission_plan_display_mode");
  if (plan_mode != fields.end()) {
    context_.mission_plan_display_mode =
      parsePlanDisplayMode(plan_mode->second, context_.mission_plan_display_mode);
  }

  applyStringField(fields, "mission_slot_categories", context_.mission_slot_categories_text);
  applyStringField(fields, "mission_high_score_category", context_.mission_high_score_category_text);
  applyStringField(fields, "mission_high_score_priority", context_.mission_high_score_priority_text);
  applyStringField(fields, "mission_cost_budget", context_.mission_cost_budget_text);
  applyStringField(fields, "mission_alpha", context_.mission_alpha_text);
  applyStringField(fields, "mission_beta", context_.mission_beta_text);
  applyStringField(fields, "mission_eta", context_.mission_eta_text);
  applyStringField(fields, "mission_g_pick_place", context_.mission_g_pick_place_text);
  applyStringField(fields, "mission_storage_near_distance", context_.mission_storage_near_distance_text);
  applyStringField(fields, "mission_storage_far_distance", context_.mission_storage_far_distance_text);
  applyStringField(fields, "mission_return_near_distance", context_.mission_return_near_distance_text);
  applyStringField(fields, "mission_return_far_distance", context_.mission_return_far_distance_text);

  for (const auto & field : context_.param_fields) {
    if (field.value == nullptr) {
      continue;
    }
    const auto iter = fields.find("controller_" + field.name);
    if (iter == fields.end()) {
      continue;
    }
    double parsed = 0.0;
    if (parseDoubleScalar(iter->second, parsed) && (!field.positive || parsed > 0.0)) {
      *field.value = parsed;
    }
  }

  RCLCPP_INFO(get_logger(), "Loaded persisted UI state: %s", context_.ui_state_file.c_str());
}

void NavigationMapNode::savePersistentUiStateIfChanged(bool force)
{
  const auto snapshot = serializePersistentUiState(context_);
  if (!force && snapshot == persisted_ui_state_snapshot_) {
    return;
  }

  std::string error_message;
  if (!writeTextFile(context_.ui_state_file, snapshot, &error_message)) {
    RCLCPP_WARN(get_logger(), "Failed to save UI state: %s", error_message.c_str());
    return;
  }
  persisted_ui_state_snapshot_ = snapshot;
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

  recordCommandVelocity(command);

  if (cmd_vel_publisher_ != nullptr) {
    cmd_vel_publisher_->publish(command);
  }
}

void NavigationMapNode::recordCommandVelocity(const geometry_msgs::msg::Twist & command)
{
  context_.cmd_vel_valid = true;
  context_.cmd_vel_linear_x = command.linear.x;
  context_.cmd_vel_linear_y = command.linear.y;
  context_.cmd_vel_angular_z = command.angular.z;
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

    if (context_.core_connected && !startup_stop_sent_) {
      startup_stop_sent_ = true;
      runtime_.stopNavigation("Navigation stopped at startup");
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
    savePersistentUiStateIfChanged();
    return;
  }

  const int ascii = navigation::keyboards::keyAscii(key);
  if (navigation::keyboards::isEscKey(key) || ascii == 'q' || ascii == 'Q') {
    savePersistentUiStateIfChanged(true);
    rclcpp::shutdown();
  } else if (ascii == 'c' || ascii == 'C') {
    points_workflow_.clearPoints();
  }
  savePersistentUiStateIfChanged();
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
