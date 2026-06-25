#ifndef NAVIGATION_PARAMS_NAVIGATION_PARAMS_HPP_
#define NAVIGATION_PARAMS_NAVIGATION_PARAMS_HPP_

#include <string>
#include <vector>

#include "controller.hpp"
#include "rclcpp/node.hpp"

namespace navigation::params
{

struct RuntimeConfig
{
  std::string source{"simulation"};
  std::string robot_name{"blackW"};
  std::string scene{"terrain"};
  std::string window_name{"blackW top view navigation"};
  std::string points_file;
  bool show_window{true};
  std::string cmd_vel_topic{"/cmd_vel"};
  int ui_size{10};
  int map_width_px{0};
  int map_height_px{0};
  double map_padding_px{20.0};
  bool panel_collapsed{true};
  double update_rate_hz{30.0};
  std::string race_logic{"obstacle"};
  double mission_task_radius{0.40};
  std::string mission_resume_event{"completed"};
  std::string mission_pickup_resume_event{"completed"};
  std::string mission_place_resume_event{"completed"};
  std::string arm_mission_service{"/arm/mission_event"};
  std::string navigation_arm_event_service{"/navigation/arm_event"};
  double mission_arm_retry_period{1.0};
  ControllerConfig controller_config;
};

struct ParamField
{
  std::string name;
  double * value{nullptr};
  bool positive{false};
};

std::vector<ParamField> makeControllerParamFields(ControllerConfig & config);
RuntimeConfig declareRuntimeConfig(rclcpp::Node & node);
std::string defaultParamsName();
std::string controllerParamDirName(const std::string & controller_name);
std::string defaultParamsFilePath(const std::string & default_points_file);
std::string resolveParamsFilePath(
  const std::string & default_points_file,
  const std::string & controller_name,
  const std::string & path_or_name);
std::vector<std::string> listParamsFiles(
  const std::string & default_points_file,
  const std::string & controller_name);
bool saveParamsFile(
  const std::string & path,
  const std::vector<ParamField> & fields,
  std::string * error_message);
bool loadParamsFile(
  const std::string & path,
  const std::vector<ParamField> & fields,
  std::string * error_message);

}  // namespace navigation::params

#endif  // NAVIGATION_PARAMS_NAVIGATION_PARAMS_HPP_
