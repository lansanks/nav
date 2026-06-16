#include "app/navigation_ui_coordinator.hpp"

#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

#include "calibration/radar_calibration.hpp"
#include "keyboards/navigation_input_handler.hpp"
#include "keyboards/navigation_keys.hpp"
#include "maps/navigation_map_helpers.hpp"
#include "maps/point_store.hpp"
#include "params/navigation_params.hpp"

namespace navigation::app
{

NavigationUiCoordinator::NavigationUiCoordinator(
  NavigationNodeContext & context,
  NavigationRuntime & runtime,
  NavigationPointsWorkflow & points_workflow,
  rclcpp::Node & node,
  rclcpp::Logger logger)
: context_(context),
  runtime_(runtime),
  points_workflow_(points_workflow),
  node_(node),
  logger_(logger)
{
}

void NavigationUiCoordinator::handleUiHit(const navigation::ui::MapUiHit & hit)
{
  if (hit.action == navigation::ui::MapUiAction::DropdownOption) {
    selectDropdownOption(hit.option_index);
    return;
  }

  handleUiAction(hit.action);
}

void NavigationUiCoordinator::handleUiAction(navigation::ui::MapUiAction action)
{
  switch (action) {
    case navigation::ui::MapUiAction::LoadPoints:
      toggleDropdown(navigation::ui::MapDropdownMode::LoadPoints);
      break;
    case navigation::ui::MapUiAction::NewPoints:
      beginTextInput(
        navigation::keyboards::TextInputMode::NewPoints,
        "New point file name",
        navigation::maps::defaultNewPointsName());
      break;
    case navigation::ui::MapUiAction::ChooseMap:
      toggleDropdown(navigation::ui::MapDropdownMode::ChooseMap);
      break;
    case navigation::ui::MapUiAction::SavePointsAs:
      beginTextInput(
        navigation::keyboards::TextInputMode::SavePointsAs,
        "Save point file as",
        std::filesystem::path(context_.points_file).filename().string());
      break;
    case navigation::ui::MapUiAction::StartNavigation:
      clearDropdown();
      if (runtime_.isNavigationActive()) {
        runtime_.stopNavigation("Navigation stopped");
      } else {
        toggleDropdown(navigation::ui::MapDropdownMode::ChooseController);
      }
      break;
    case navigation::ui::MapUiAction::OnlineParams:
      clearDropdown();
      context_.params_session.open(context_.status_message);
      break;
    case navigation::ui::MapUiAction::Radar:
      clearDropdown();
      context_.params_session.setActive(false);
      context_.radar_popup_active = true;
      context_.radar_result_pending = false;
      context_.status_message = "Radar calibration";
      break;
    case navigation::ui::MapUiAction::ParamSave:
      beginTextInput(
        navigation::keyboards::TextInputMode::SaveParamsAs,
        "Save params as",
        navigation::params::defaultParamsName());
      break;
    case navigation::ui::MapUiAction::ParamLoad:
      context_.params_session.setActive(false);
      toggleDropdown(navigation::ui::MapDropdownMode::LoadParams);
      break;
    case navigation::ui::MapUiAction::ParamClose:
      context_.params_session.setActive(false);
      context_.status_message = "Param editor closed";
      break;
    case navigation::ui::MapUiAction::InputClose:
      cancelTextInput();
      break;
    case navigation::ui::MapUiAction::RadarListen:
      toggleRadarListener();
      break;
    case navigation::ui::MapUiAction::RadarSavePoint:
      if (!context_.radar_listener.getState(context_.radar_latest_state)) {
        context_.status_message = "No radar pose";
        break;
      }
      if (context_.radar_save_file_confirmed && !context_.radar_data_file.empty()) {
        saveRadarPointAs(context_.radar_data_file);
        break;
      }
      beginTextInput(
        navigation::keyboards::TextInputMode::SaveRadarPointAs,
        "Save radar point file",
        context_.radar_data_file.empty() ? std::string("radar_points.yaml") :
        std::filesystem::path(context_.radar_data_file).filename().string());
      break;
    case navigation::ui::MapUiAction::RadarSelectDataFile:
      toggleDropdown(navigation::ui::MapDropdownMode::RadarDataFile);
      break;
    case navigation::ui::MapUiAction::RadarSelectPointsFile:
      toggleDropdown(navigation::ui::MapDropdownMode::RadarPointsFile);
      break;
    case navigation::ui::MapUiAction::RadarRegister:
      runRadarRegistration();
      break;
    case navigation::ui::MapUiAction::RadarAcceptCalibration:
      acceptRadarCalibration();
      break;
    case navigation::ui::MapUiAction::RadarRejectCalibration:
      rejectRadarCalibration();
      break;
    case navigation::ui::MapUiAction::RadarClose:
      closeRadarPopup();
      break;
    case navigation::ui::MapUiAction::ToggleTheme:
      context_.light_theme = !context_.light_theme;
      context_.status_message = context_.light_theme ? "Light theme" : "Dark theme";
      break;
    case navigation::ui::MapUiAction::ToggleFullscreen:
      toggleFullscreen();
      break;
    case navigation::ui::MapUiAction::ToggleRaceLogic:
      context_.race_logic = context_.race_logic == "mission" ? "obstacle" : "mission";
      clearDropdown();
      runtime_.stopNavigationForRouteChange();
      runtime_.syncControllerWaypoints();
      context_.status_message = context_.race_logic == "mission" ? "Mission race logic" : "Obstacle race logic";
      break;
    case navigation::ui::MapUiAction::TogglePanel:
      togglePanel();
      break;
    case navigation::ui::MapUiAction::UiOnly:
      if (context_.dropdown_mode != navigation::ui::MapDropdownMode::None) {
        clearDropdown();
        context_.status_message = "Selection cancelled";
      }
      break;
    case navigation::ui::MapUiAction::DropdownOption:
    case navigation::ui::MapUiAction::ParamOption:
    case navigation::ui::MapUiAction::None:
      break;
  }
}

void NavigationUiCoordinator::toggleFullscreen()
{
  context_.fullscreen = !context_.fullscreen;
  context_.fullscreen_dirty = true;
  context_.status_message = context_.fullscreen ? "Fullscreen" : "Windowed";
}

void NavigationUiCoordinator::togglePanel()
{
  context_.panel_collapsed = !context_.panel_collapsed;
  clearDropdown();
  context_.status_message = context_.panel_collapsed ? "Controls hidden" : "Controls shown";
}

void NavigationUiCoordinator::toggleDropdown(navigation::ui::MapDropdownMode mode)
{
  if (context_.dropdown_mode == mode) {
    clearDropdown();
    context_.status_message = "Selection cancelled";
    return;
  }

  beginDropdown(mode);
}

void NavigationUiCoordinator::beginDropdown(navigation::ui::MapDropdownMode mode)
{
  resetTextInput();
  context_.dropdown_mode = mode;
  context_.dropdown_paths.clear();

  if (mode == navigation::ui::MapDropdownMode::LoadPoints) {
    context_.dropdown_paths = navigation::maps::listPointsFiles(context_.points_file);
    navigation::maps::addExistingPath(context_.dropdown_paths, context_.points_file);
    context_.dropdown_selected_index =
      navigation::maps::findPathIndex(context_.dropdown_paths, context_.points_file);
    context_.status_message = context_.dropdown_paths.empty() ? "No point files found" : "Select point file";
  } else if (mode == navigation::ui::MapDropdownMode::ChooseMap) {
    try {
      context_.dropdown_paths = navigation::maps::listSceneFiles(context_.robot_name);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Failed to list map files: %s", error.what());
      context_.dropdown_paths.clear();
    }
    navigation::maps::addExistingPath(context_.dropdown_paths, context_.current_map_file);
    context_.dropdown_selected_index =
      navigation::maps::findPathIndex(context_.dropdown_paths, context_.current_map_file);
    context_.status_message = context_.dropdown_paths.empty() ? "No map files found" : "Select map file";
  } else if (mode == navigation::ui::MapDropdownMode::ChooseController) {
    context_.dropdown_paths = context_.controller_names;
    context_.dropdown_selected_index =
      navigation::maps::findPathIndex(context_.dropdown_paths, context_.selected_controller_name);
    context_.status_message = context_.dropdown_paths.empty() ? "No controllers found" : "Select controller";
  } else if (mode == navigation::ui::MapDropdownMode::LoadParams) {
    context_.dropdown_paths = navigation::params::listParamsFiles(
      navigation::maps::defaultPointsFilePath(),
      context_.selected_controller_name);
    context_.dropdown_selected_index = context_.dropdown_paths.empty() ? -1 : 0;
    context_.status_message = context_.dropdown_paths.empty() ? "No param files found" : "Select params file";
  } else if (mode == navigation::ui::MapDropdownMode::RadarDataFile) {
    context_.dropdown_paths = navigation::calibration::listCalibrationDataFiles();
    navigation::maps::addExistingPath(context_.dropdown_paths, context_.radar_data_file);
    context_.dropdown_selected_index =
      navigation::maps::findPathIndex(context_.dropdown_paths, context_.radar_data_file);
    context_.status_message = context_.dropdown_paths.empty() ? "No radar files found" : "Select radar points file";
  } else if (mode == navigation::ui::MapDropdownMode::RadarPointsFile) {
    context_.dropdown_paths = navigation::maps::listPointsFiles(context_.points_file);
    navigation::maps::addExistingPath(context_.dropdown_paths, context_.points_file);
    navigation::maps::addExistingPath(context_.dropdown_paths, context_.radar_points_file);
    context_.dropdown_selected_index =
      navigation::maps::findPathIndex(context_.dropdown_paths, context_.radar_points_file);
    context_.status_message = context_.dropdown_paths.empty() ? "No point files found" : "Select map points file";
  }

  if (context_.dropdown_selected_index < 0 && !context_.dropdown_paths.empty()) {
    context_.dropdown_selected_index = 0;
  }
  context_.dropdown_labels = navigation::maps::makePathLabels(context_.dropdown_paths);
}

void NavigationUiCoordinator::clearDropdown()
{
  context_.dropdown_mode = navigation::ui::MapDropdownMode::None;
  context_.dropdown_paths.clear();
  context_.dropdown_labels.clear();
  context_.dropdown_selected_index = -1;
}

void NavigationUiCoordinator::selectDropdownOption(int option_index)
{
  if (option_index < 0 || option_index >= static_cast<int>(context_.dropdown_paths.size())) {
    return;
  }

  const auto mode = context_.dropdown_mode;
  const auto selected_path = context_.dropdown_paths[static_cast<std::size_t>(option_index)];
  clearDropdown();

  if (mode == navigation::ui::MapDropdownMode::LoadPoints) {
    points_workflow_.loadPointsFromFile(selected_path);
  } else if (mode == navigation::ui::MapDropdownMode::ChooseMap) {
    loadMapFile(selected_path);
  } else if (mode == navigation::ui::MapDropdownMode::ChooseController) {
    runtime_.startNavigation(selected_path);
  } else if (mode == navigation::ui::MapDropdownMode::LoadParams) {
    loadParamsFile(selected_path);
    context_.params_session.setActive(true);
  } else if (mode == navigation::ui::MapDropdownMode::RadarDataFile) {
    context_.radar_data_file = selected_path;
    context_.radar_save_file_confirmed = true;
    context_.status_message = "Radar file selected";
  } else if (mode == navigation::ui::MapDropdownMode::RadarPointsFile) {
    context_.radar_points_file = selected_path;
    context_.status_message = "Map file selected";
  }
}

void NavigationUiCoordinator::resetTextInput()
{
  context_.input_mode = navigation::keyboards::TextInputMode::None;
  context_.input_label.clear();
  context_.input_text.clear();
}

void NavigationUiCoordinator::beginTextInput(
  navigation::keyboards::TextInputMode mode,
  const std::string & label,
  const std::string & default_text)
{
  clearDropdown();
  context_.input_mode = mode;
  context_.input_label = label;
  context_.input_text = default_text;
  context_.status_message = "Typing input";
}

void NavigationUiCoordinator::cancelTextInput()
{
  resetTextInput();
  context_.status_message = "Input cancelled";
}

void NavigationUiCoordinator::confirmTextInput()
{
  const auto value = context_.input_text;
  const auto mode = context_.input_mode;
  resetTextInput();

  if (value.empty()) {
    context_.status_message = "Input is empty";
    return;
  }

  if (mode == navigation::keyboards::TextInputMode::NewPoints) {
    points_workflow_.createNewPointsFile(value);
  } else if (mode == navigation::keyboards::TextInputMode::SavePointsAs) {
    points_workflow_.savePointsAs(value);
  } else if (mode == navigation::keyboards::TextInputMode::SaveParamsAs) {
    saveParamsAs(value);
  } else if (mode == navigation::keyboards::TextInputMode::SaveRadarPointAs) {
    saveRadarPointAs(value);
  }
}

bool NavigationUiCoordinator::handleTextInputKey(int key)
{
  return navigation::keyboards::handleTextInputKey(
    key,
    context_.input_mode,
    context_.input_text,
    [this]() {
      cancelTextInput();
    },
    [this]() {
      confirmTextInput();
    });
}

bool NavigationUiCoordinator::handleParamsKey(int key)
{
  return context_.params_session.handleKey(
    key,
    context_.param_fields,
    [this]() { runtime_.applyControllerConfig(); },
    [this]() {
      beginTextInput(
        navigation::keyboards::TextInputMode::SaveParamsAs,
        "Save params as",
        navigation::params::defaultParamsName());
    },
    [this]() {
      toggleDropdown(navigation::ui::MapDropdownMode::LoadParams);
    },
    context_.status_message);
}

bool NavigationUiCoordinator::handleDropdownKey(int key)
{
  return navigation::keyboards::handleDropdownKey(
    key,
    context_.dropdown_mode,
    static_cast<int>(context_.dropdown_paths.size()),
    context_.dropdown_selected_index,
    [this]() {
      clearDropdown();
      context_.status_message = "Selection cancelled";
    },
    [this](int option_index) {
      selectDropdownOption(option_index);
    });
}

bool NavigationUiCoordinator::handleActiveInputKey(int key)
{
  if (handleTextInputKey(key)) {
    return true;
  }
  if (context_.radar_popup_active && key != -1) {
    if (navigation::keyboards::isEscKey(key)) {
      if (context_.radar_result_pending) {
        rejectRadarCalibration();
      } else {
        closeRadarPopup();
      }
      return true;
    }
    if (context_.radar_result_pending && navigation::keyboards::isEnterKey(key)) {
      acceptRadarCalibration();
      return true;
    }
  }
  if (handleParamsKey(key)) {
    return true;
  }
  return handleDropdownKey(key);
}

navigation::ui::MapUiState NavigationUiCoordinator::buildUiState()
{
  refreshRadarState();

  navigation::ui::MapUiState ui_state;
  ui_state.map_file = context_.current_map_file;
  ui_state.points_file = context_.points_file;
  ui_state.controller_name = context_.selected_controller_name;
  ui_state.navigation_status = context_.navigation_status;
  ui_state.race_logic = context_.race_logic;
  if (context_.controller != nullptr) {
    const auto controller_status = context_.controller->status();
    ui_state.navigation_active = controller_status.active;
    ui_state.navigation_target_index = controller_status.target_index;
    ui_state.navigation_point_count = controller_status.point_count;
    if (!controller_status.message.empty() && ui_state.navigation_status.empty()) {
      ui_state.navigation_status = controller_status.message;
    }
  } else if (context_.remote_control) {
    ui_state.navigation_active = context_.remote_navigation_active;
    ui_state.navigation_target_index = context_.remote_navigation_target_index;
    ui_state.navigation_point_count = context_.remote_navigation_point_count;
  }
  ui_state.message = context_.status_message;
  ui_state.input_active = context_.input_mode != navigation::keyboards::TextInputMode::None;
  ui_state.input_label = context_.input_label;
  ui_state.input_text = context_.input_text;
  context_.params_session.applyToState(ui_state, context_.param_fields);
  ui_state.radar_active = context_.radar_popup_active;
  ui_state.radar_listening = context_.radar_listener.active();
  ui_state.radar_pose_valid = context_.radar_latest_state.valid;
  ui_state.radar_confirm_active = context_.radar_result_pending;
  ui_state.radar_result_unstable = context_.radar_pending_result.unstable;
  ui_state.light_theme = context_.light_theme;
  ui_state.radar_topic = context_.radar_topic;
  ui_state.radar_pose_text = formatRadarPoseText();
  ui_state.radar_data_file = context_.radar_data_file.empty() ?
    std::string() :
    std::filesystem::path(context_.radar_data_file).filename().string();
  ui_state.radar_points_file = context_.radar_points_file.empty() ?
    std::string() :
    std::filesystem::path(context_.radar_points_file).filename().string();
  ui_state.radar_result_summary = formatRadarResultSummary();
  ui_state.radar_transform_text = formatRadarTransformText();
  ui_state.dropdown_mode = context_.dropdown_mode;
  ui_state.dropdown_items = context_.dropdown_labels;
  ui_state.dropdown_selected_index = context_.dropdown_selected_index;
  ui_state.panel_scroll_px = context_.panel_scroll_px;
  ui_state.panel_collapsed = context_.panel_collapsed;
  ui_state.fullscreen = context_.fullscreen;
  ui_state.core_connected = !context_.remote_control || context_.core_connected;
  ui_state.cmd_vel_valid = context_.cmd_vel_valid;
  ui_state.cmd_vel_linear_x = context_.cmd_vel_linear_x;
  ui_state.cmd_vel_linear_y = context_.cmd_vel_linear_y;
  ui_state.cmd_vel_angular_z = context_.cmd_vel_angular_z;
  return ui_state;
}

bool NavigationUiCoordinator::saveParamsFile(const std::string & path)
{
  std::string error_message;
  if (!navigation::params::saveParamsFile(path, context_.param_fields, &error_message)) {
    context_.status_message = "Param save failed";
    RCLCPP_ERROR(logger_, "Failed to save navigation params: %s", error_message.c_str());
    return false;
  }

  context_.status_message = "Params saved";
  RCLCPP_INFO(logger_, "Saved navigation params: %s", path.c_str());
  return true;
}

bool NavigationUiCoordinator::loadParamsFile(const std::string & path)
{
  std::string error_message;
  if (!navigation::params::loadParamsFile(path, context_.param_fields, &error_message)) {
    context_.status_message = "No params file";
    RCLCPP_WARN(logger_, "Failed to load navigation params: %s", error_message.c_str());
    return false;
  }

  runtime_.applyControllerConfig();
  context_.status_message = "Params loaded";
  RCLCPP_INFO(logger_, "Loaded navigation params: %s", path.c_str());
  return true;
}

void NavigationUiCoordinator::saveParamsAs(const std::string & path_or_name)
{
  const auto path = navigation::params::resolveParamsFilePath(
    navigation::maps::defaultPointsFilePath(),
    context_.selected_controller_name,
    path_or_name);
  if (saveParamsFile(path)) {
    context_.params_session.setActive(true);
  }
}

void NavigationUiCoordinator::loadMapFile(const std::string & path_or_scene)
{
  std::string scene_path;
  const std::filesystem::path requested(path_or_scene);
  if (std::filesystem::exists(requested)) {
    scene_path = requested.string();
  } else {
    scene_path = navigation::maps::resolveScenePath(context_.robot_name, path_or_scene);
  }

  try {
    context_.map->load(scene_path);
  } catch (const std::exception & error) {
    context_.status_message = "Map load failed";
    RCLCPP_ERROR(logger_, "Failed to load map '%s': %s", scene_path.c_str(), error.what());
    return;
  }

  context_.current_map_file = scene_path;
  context_.status_message = "Loaded map: " + std::filesystem::path(scene_path).filename().string();
  RCLCPP_INFO(logger_, "Loaded map file: %s", context_.current_map_file.c_str());
}

void NavigationUiCoordinator::toggleRadarListener()
{
  if (context_.radar_listener.active()) {
    context_.radar_listener.stop();
    context_.radar_latest_state = navigation::RobotNavigationState{};
    context_.radar_save_file_confirmed = false;
    context_.status_message = "Radar listener stopped";
    return;
  }

  context_.radar_listener.start(node_, context_.radar_topic);
  context_.radar_save_file_confirmed = false;
  context_.status_message = "Listening for radar";
}

void NavigationUiCoordinator::saveRadarPointAs(const std::string & path_or_name)
{
  navigation::RobotNavigationState state;
  if (!context_.radar_listener.getState(state)) {
    context_.status_message = "No radar pose";
    RCLCPP_WARN(logger_, "Cannot save radar calibration point: no radar pose received.");
    return;
  }

  navigation::maps::MapPoint point;
  point.x = state.x;
  point.y = state.y;
  point.fast = false;

  std::string saved_path;
  std::string error_message;
  if (!navigation::calibration::appendRadarPoint(path_or_name, point, &saved_path, &error_message)) {
    context_.status_message = "Radar point save failed";
    RCLCPP_ERROR(logger_, "Failed to save radar point: %s", error_message.c_str());
    return;
  }

  context_.radar_data_file = saved_path;
  context_.radar_save_file_confirmed = true;
  context_.status_message = "Radar point saved";
  RCLCPP_INFO(logger_, "Saved radar point x=%.6f y=%.6f to %s", point.x, point.y, saved_path.c_str());
}

void NavigationUiCoordinator::runRadarRegistration()
{
  if (context_.radar_data_file.empty()) {
    context_.status_message = "Select radar file";
    beginDropdown(navigation::ui::MapDropdownMode::RadarDataFile);
    return;
  }
  if (context_.radar_points_file.empty()) {
    context_.status_message = "Select map file";
    beginDropdown(navigation::ui::MapDropdownMode::RadarPointsFile);
    return;
  }

  const auto radar_points = navigation::maps::loadPointsFile(context_.radar_data_file);
  const auto mujoco_points = navigation::maps::loadPointsFile(context_.radar_points_file);
  std::string error_message;
  navigation::calibration::KabschResult result;
  if (!navigation::calibration::computeKabsch(radar_points, mujoco_points, result, &error_message)) {
    context_.status_message = "Registration failed: " + error_message;
    RCLCPP_WARN(logger_, "%s", context_.status_message.c_str());
    return;
  }

  context_.radar_pending_result = result;
  context_.radar_result_pending = true;
  clearDropdown();
  context_.status_message = result.unstable ? "Registration unstable" : "Registration ready";
}

void NavigationUiCoordinator::acceptRadarCalibration()
{
  if (!context_.radar_result_pending) {
    return;
  }

  const auto path = navigation::calibration::resolveCalibrationParamsFilePath(
    context_.radar_data_file,
    context_.radar_points_file);
  std::string error_message;
  if (!navigation::calibration::saveCalibrationParams(
      path,
      context_.radar_data_file,
      context_.radar_points_file,
      context_.radar_pending_result,
      &error_message))
  {
    context_.status_message = "Calibration save failed";
    RCLCPP_ERROR(logger_, "Failed to save calibration params: %s", error_message.c_str());
    return;
  }

  context_.radar_result_pending = false;
  context_.status_message = "Calibration saved";
  RCLCPP_INFO(logger_, "Saved radar calibration params: %s", path.c_str());
}

void NavigationUiCoordinator::rejectRadarCalibration()
{
  context_.radar_result_pending = false;
  context_.status_message = "Calibration rejected";
}

void NavigationUiCoordinator::closeRadarPopup()
{
  clearDropdown();
  context_.radar_popup_active = false;
  context_.radar_result_pending = false;
  context_.status_message = "Radar calibration closed";
}

void NavigationUiCoordinator::refreshRadarState()
{
  navigation::RobotNavigationState state;
  if (context_.radar_listener.getState(state)) {
    context_.radar_latest_state = state;
  } else if (!context_.radar_listener.active()) {
    context_.radar_latest_state = navigation::RobotNavigationState{};
  }
}

std::string NavigationUiCoordinator::formatRadarPoseText() const
{
  if (!context_.radar_latest_state.valid) {
    return "Pose: waiting";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(3)
         << "Pose: x=" << context_.radar_latest_state.x
         << " y=" << context_.radar_latest_state.y
         << " yaw=" << context_.radar_latest_state.yaw;
  return output.str();
}

std::string NavigationUiCoordinator::formatRadarResultSummary() const
{
  if (!context_.radar_result_pending) {
    return "";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(4)
         << "pairs=" << context_.radar_pending_result.errors.size()
         << " mean=" << context_.radar_pending_result.mean_error
         << " max=" << context_.radar_pending_result.max_error
         << " yaw=" << context_.radar_pending_result.yaw_offset;
  return output.str();
}

std::string NavigationUiCoordinator::formatRadarTransformText() const
{
  if (!context_.radar_result_pending) {
    return "";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(4)
         << "t=(" << context_.radar_pending_result.tx
         << ", " << context_.radar_pending_result.ty
         << ") R=[[" << context_.radar_pending_result.r00
         << ", " << context_.radar_pending_result.r01
         << "], [" << context_.radar_pending_result.r10
         << ", " << context_.radar_pending_result.r11 << "]]";
  return output.str();
}

}  // namespace navigation::app
