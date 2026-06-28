#include "params/navigation_params.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <system_error>

#include "maps/point_store.hpp"

namespace navigation::params
{
namespace
{

void setError(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

std::string trim(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::filesystem::path paramsDirectory(
  const std::string & default_points_file,
  const std::string & controller_name)
{
  const auto config_dir = std::filesystem::path(default_points_file).parent_path().parent_path();
  return config_dir / "params" / controllerParamDirName(controller_name);
}

double positiveOrDefault(double value, double fallback)
{
  return value > 0.0 ? value : fallback;
}

std::string normalizedChoiceOrDefault(
  std::string value,
  const std::vector<std::string> & allowed,
  const std::string & fallback)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return std::find(allowed.begin(), allowed.end(), value) == allowed.end() ? fallback : value;
}

std::string normalizedResumeEventOrDefault(
  rclcpp::Node & node,
  const std::string & parameter_name,
  const std::vector<std::string> & allowed,
  const std::string & fallback)
{
  return normalizedChoiceOrDefault(
    node.declare_parameter<std::string>(parameter_name, fallback),
    allowed,
    fallback);
}

int clampUiSize(int value)
{
  return std::clamp(value, 5, 15);
}

}  // namespace

std::vector<ParamField> makeControllerParamFields(ControllerConfig & config)
{
  return {
    {"waypoint_tolerance", &config.waypoint_tolerance, true},
    {"max_linear_speed", &config.max_linear_speed, true},
    {"max_angular_speed", &config.max_angular_speed, true},
    {"k_rho", &config.k_rho, false},
    {"k_alpha", &config.k_alpha, false},
    {"k_beta", &config.k_beta, false},
    {"fast_max_linear_speed", &config.fast_max_linear_speed, true},
    {"fast_max_angular_speed", &config.fast_max_angular_speed, true},
    {"fast_k_rho", &config.fast_k_rho, false},
    {"fast_k_alpha", &config.fast_k_alpha, false},
    {"fast_k_beta", &config.fast_k_beta, false},
  };
}

RuntimeConfig declareRuntimeConfig(rclcpp::Node & node)
{
  RuntimeConfig config;
  config.source = node.declare_parameter<std::string>("source", config.source);
  config.robot_name = node.declare_parameter<std::string>("robot_name", config.robot_name);
  config.scene = node.declare_parameter<std::string>("scene", config.scene);
  config.window_name = node.declare_parameter<std::string>("window_name", config.window_name);
  config.points_file = node.declare_parameter<std::string>("points_file", "");
  if (config.points_file.empty()) {
    config.points_file = navigation::maps::defaultPointsFilePath();
  } else {
    config.points_file = navigation::maps::resolvePointsFilePath(config.points_file);
  }
  config.show_window = node.declare_parameter<bool>("show_window", config.show_window);
  config.cmd_vel_topic = node.declare_parameter<std::string>("cmd_vel_topic", config.cmd_vel_topic);

  auto & controller_config = config.controller_config;
  controller_config.waypoint_tolerance =
    positiveOrDefault(node.declare_parameter<double>("waypoint_tolerance", 0.20), 0.20);
  controller_config.max_linear_speed =
    positiveOrDefault(node.declare_parameter<double>("max_linear_speed", 0.70), 0.70);
  controller_config.max_angular_speed =
    positiveOrDefault(node.declare_parameter<double>("max_angular_speed", 1.80), 1.80);
  controller_config.k_rho = node.declare_parameter<double>("k_rho", 1.20);
  controller_config.k_alpha = node.declare_parameter<double>("k_alpha", 2.40);
  controller_config.k_beta = node.declare_parameter<double>("k_beta", -0.60);
  controller_config.fast_max_linear_speed =
    positiveOrDefault(node.declare_parameter<double>("fast_max_linear_speed", 1.20), 1.20);
  controller_config.fast_max_angular_speed =
    positiveOrDefault(node.declare_parameter<double>("fast_max_angular_speed", 2.20), 2.20);
  controller_config.fast_k_rho = node.declare_parameter<double>("fast_k_rho", 1.60);
  controller_config.fast_k_alpha = node.declare_parameter<double>("fast_k_alpha", 2.80);
  controller_config.fast_k_beta = node.declare_parameter<double>("fast_k_beta", -0.70);

  config.ui_size = clampUiSize(node.declare_parameter<int>("ui_size", config.ui_size));
  config.map_width_px = node.declare_parameter<int>("map_width_px", config.map_width_px);
  config.map_height_px = node.declare_parameter<int>("map_height_px", config.map_height_px);
  if (config.map_width_px <= 0) {
    config.map_width_px = config.ui_size * 110;
  }
  if (config.map_height_px <= 0) {
    config.map_height_px = config.ui_size * 82;
  }
  config.map_padding_px = node.declare_parameter<double>("map_padding_px", config.map_padding_px);
  config.panel_collapsed = node.declare_parameter<bool>("panel_collapsed", config.panel_collapsed);
  config.update_rate_hz = node.declare_parameter<double>("update_rate_hz", config.update_rate_hz);
  config.update_rate_hz = positiveOrDefault(config.update_rate_hz, 30.0);
  config.race_logic = normalizedChoiceOrDefault(
    node.declare_parameter<std::string>("race_logic", config.race_logic),
    {"obstacle", "mission"},
    "obstacle");
  config.mission_task_radius =
    positiveOrDefault(node.declare_parameter<double>("mission_task_radius", config.mission_task_radius), 0.40);
  config.mission_resume_event = normalizedChoiceOrDefault(
    node.declare_parameter<std::string>("mission_resume_event", config.mission_resume_event),
    {"ack", "grabbed", "completed"},
    "completed");
  config.mission_pickup_resume_event = normalizedResumeEventOrDefault(
    node,
    "mission_pickup_resume_event",
    {"ack", "grabbed", "completed"},
    config.mission_resume_event);
  config.mission_place_resume_event = normalizedResumeEventOrDefault(
    node,
    "mission_place_resume_event",
    {"ack", "placed", "completed"},
    config.mission_resume_event == "grabbed" ? "placed" : config.mission_resume_event);
  config.arm_mission_service =
    node.declare_parameter<std::string>("arm_mission_service", config.arm_mission_service);
  config.navigation_arm_event_service =
    node.declare_parameter<std::string>(
    "navigation_arm_event_service",
    config.navigation_arm_event_service);
  config.mission_arm_retry_period =
    positiveOrDefault(
    node.declare_parameter<double>("mission_arm_retry_period", config.mission_arm_retry_period),
    1.0);
  config.navigation_event_wait_seconds =
    positiveOrDefault(
    node.declare_parameter<double>("navigation_event_wait_seconds", config.navigation_event_wait_seconds),
    1.0);
  config.rl_debug_key_topic =
    node.declare_parameter<std::string>("rl_debug_key_topic", config.rl_debug_key_topic);
  return config;
}

std::string defaultParamsName()
{
  return "navigation_params.yaml";
}

std::string controllerParamDirName(const std::string & controller_name)
{
  std::string text;
  text.reserve(controller_name.size());
  for (const char ch : controller_name) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')) {
      text.push_back(ch);
    } else if (!text.empty() && text.back() != '_') {
      text.push_back('_');
    }
  }
  while (!text.empty() && text.back() == '_') {
    text.pop_back();
  }
  return text.empty() ? "controller" : text;
}

std::string defaultParamsFilePath(const std::string & default_points_file)
{
  return (paramsDirectory(default_points_file, "Sequential Waypoint") / defaultParamsName()).string();
}

std::string resolveParamsFilePath(
  const std::string & default_points_file,
  const std::string & controller_name,
  const std::string & path_or_name)
{
  if (path_or_name.empty()) {
    return (paramsDirectory(default_points_file, controller_name) / defaultParamsName()).string();
  }

  std::filesystem::path path(path_or_name);
  if (!path.has_extension()) {
    path += ".yaml";
  }

  if (path.is_absolute() || path.has_parent_path()) {
    return path.string();
  }

  return (paramsDirectory(default_points_file, controller_name) / path).string();
}

std::vector<std::string> listParamsFiles(
  const std::string & default_points_file,
  const std::string & controller_name)
{
  std::vector<std::string> files;
  const auto dir = paramsDirectory(default_points_file, controller_name);
  std::error_code error;
  if (!std::filesystem::exists(dir, error)) {
    return files;
  }

  for (const auto & entry : std::filesystem::directory_iterator(dir, error)) {
    if (error) {
      break;
    }
    if (!entry.is_regular_file(error)) {
      continue;
    }
    const auto extension = entry.path().extension().string();
    if (extension == ".yaml" || extension == ".yml") {
      files.push_back(entry.path().lexically_normal().string());
    }
  }

  std::sort(files.begin(), files.end(), [](const std::string & left, const std::string & right) {
    return std::filesystem::path(left).filename().string() <
      std::filesystem::path(right).filename().string();
  });
  return files;
}

bool saveParamsFile(
  const std::string & path,
  const std::vector<ParamField> & fields,
  std::string * error_message)
{
  const std::filesystem::path file_path(path);
  const auto parent = file_path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      setError(error_message, "failed to create param directory '" + parent.string() + "': " + error.message());
      return false;
    }
  }

  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) {
    setError(error_message, "failed to open param file for writing: " + path);
    return false;
  }

  output << std::fixed << std::setprecision(6);
  for (const auto & field : fields) {
    output << field.name << ": " << *field.value << "\n";
  }
  return true;
}

bool loadParamsFile(
  const std::string & path,
  const std::vector<ParamField> & fields,
  std::string * error_message)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    setError(error_message, "params file not found: " + path);
    return false;
  }

  std::string line;
  while (std::getline(input, line)) {
    auto text = trim(line);
    if (text.empty() || text[0] == '#') {
      continue;
    }

    const auto sep = text.find(':');
    if (sep == std::string::npos) {
      continue;
    }

    const auto key = trim(text.substr(0, sep));
    const auto value_text = trim(text.substr(sep + 1));
    try {
      const double value = std::stod(value_text);
      for (const auto & field : fields) {
        if (field.name == key && (!field.positive || value > 0.0)) {
          *field.value = value;
          break;
        }
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  return true;
}

}  // namespace navigation::params
