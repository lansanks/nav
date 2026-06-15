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
  ToggleRaceLogic,
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
  bool core_connected{true};
};

}  // namespace navigation::ui

#endif  // NAVIGATION_UI_MAP_UI_TYPES_HPP_
