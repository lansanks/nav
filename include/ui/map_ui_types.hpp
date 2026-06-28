#ifndef NAVIGATION_UI_MAP_UI_TYPES_HPP_
#define NAVIGATION_UI_MAP_UI_TYPES_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace navigation::ui
{

enum class MapUiAction
{
  None,
  LoadPoints,
  NewPoints,
  ChooseMap,
  SavePointsAs,
  StartNavigation,
  OnlineParams,
  Radar,
  ParamOption,
  ParamSave,
  ParamLoad,
  ParamClose,
  InputClose,
  RadarListen,
  RadarSavePoint,
  RadarSelectDataFile,
  RadarSelectPointsFile,
  RadarRegister,
  RadarClose,
  RadarAcceptCalibration,
  RadarRejectCalibration,
  ToggleTheme,
  ToggleFullscreen,
  ToggleMissionPlan,
  ToggleRaceLogic,
  Settings,
  SettingsField,
  SettingsApply,
  SettingsClose,
  DropdownOption,
  TogglePanel,
  UiOnly,
};

enum class MapDropdownMode
{
  None,
  LoadPoints,
  ChooseMap,
  ChooseController,
  LoadParams,
  RadarDataFile,
  RadarPointsFile,
};

struct MapUiHit
{
  MapUiAction action{MapUiAction::None};
  int option_index{-1};
};

enum class MapPlanDisplayMode
{
  Full,
  OrderOnly,
  Hidden,
};

struct MapPlanPoint
{
  double x{0.0};
  double y{0.0};
  bool loaded_segment_to_here{false};
};

struct MapUiState
{
  std::string map_file;
  std::string points_file;
  std::string controller_name;
  std::string navigation_status;
  std::string race_logic{"obstacle"};
  std::string message;
  std::string input_label;
  std::string input_text;
  std::vector<std::string> param_names;
  std::vector<std::string> param_values;
  std::size_t param_selected_index{0};
  bool params_active{false};
  bool params_editing{false};
  std::string params_edit_text;
  bool radar_active{false};
  bool radar_listening{false};
  bool radar_pose_valid{false};
  bool radar_confirm_active{false};
  bool radar_result_unstable{false};
  bool light_theme{true};
  std::string radar_topic;
  std::string radar_pose_text;
  std::string radar_data_file;
  std::string radar_points_file;
  std::string radar_result_summary;
  std::string radar_transform_text;
  bool navigation_active{false};
  std::size_t navigation_target_index{0};
  std::size_t navigation_point_count{0};
  bool input_active{false};
  MapDropdownMode dropdown_mode{MapDropdownMode::None};
  std::vector<std::string> dropdown_items;
  int dropdown_selected_index{-1};
  int panel_scroll_px{0};
  bool panel_collapsed{false};
  bool fullscreen{false};
  MapPlanDisplayMode mission_plan_display_mode{MapPlanDisplayMode::Full};
  bool core_connected{true};
  bool cmd_vel_valid{false};
  double cmd_vel_linear_x{0.0};
  double cmd_vel_linear_y{0.0};
  double cmd_vel_angular_z{0.0};
  bool settings_active{false};
  bool settings_editing{false};
  int settings_selected_index{0};
  std::string settings_edit_text;
  std::vector<std::string> settings_field_names;
  std::vector<std::string> settings_field_values;
  std::string mission_plan_summary;
  std::vector<MapPlanPoint> mission_plan_points;
  bool route_patch_active{false};
  std::size_t route_patch_insert_index{0};
  std::vector<MapPlanPoint> route_patch_points;
};

}  // namespace navigation::ui

#endif  // NAVIGATION_UI_MAP_UI_TYPES_HPP_
