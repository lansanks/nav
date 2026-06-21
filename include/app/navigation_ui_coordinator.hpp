#ifndef NAVIGATION_APP_NAVIGATION_UI_COORDINATOR_HPP_
#define NAVIGATION_APP_NAVIGATION_UI_COORDINATOR_HPP_

#include <string>

#include "app/navigation_node_context.hpp"
#include "app/navigation_points_workflow.hpp"
#include "app/navigation_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ui/map_ui_types.hpp"

namespace navigation::app
{

class NavigationUiCoordinator
{
public:
  NavigationUiCoordinator(
    NavigationNodeContext & context,
    NavigationRuntime & runtime,
    NavigationPointsWorkflow & points_workflow,
    rclcpp::Node & node,
    rclcpp::Logger logger);

  void handleUiHit(const navigation::ui::MapUiHit & hit);
  void handleUiAction(navigation::ui::MapUiAction action);
  void togglePanel();
  void toggleFullscreen();
  void toggleDropdown(navigation::ui::MapDropdownMode mode);
  void clearDropdown();
  void selectDropdownOption(int option_index);
  void beginTextInput(
    navigation::keyboards::TextInputMode mode,
    const std::string & label,
    const std::string & default_text);
  bool handleActiveInputKey(int key);
  navigation::ui::MapUiState buildUiState();

private:
  void beginDropdown(navigation::ui::MapDropdownMode mode);
  void resetTextInput();
  void cancelTextInput();
  void confirmTextInput();
  bool handleTextInputKey(int key);
  bool handleSettingsKey(int key);
  bool handleParamsKey(int key);
  bool handleDropdownKey(int key);
  void openSettings();
  void closeSettings();
  void selectSettingsField(int field_index);
  void beginSettingsEdit();
  void applySettingsEdit();
  void applyMissionSettings();
  std::vector<std::string> settingsFieldValues() const;
  std::vector<std::string> settingsFieldNames() const;
  bool saveParamsFile(const std::string & path);
  bool loadParamsFile(const std::string & path);
  void saveParamsAs(const std::string & path_or_name);
  void loadMapFile(const std::string & path_or_scene);
  void toggleRadarListener();
  void saveRadarPointAs(const std::string & path_or_name);
  void runRadarRegistration();
  void acceptRadarCalibration();
  void rejectRadarCalibration();
  void closeRadarPopup();
  void refreshRadarState();
  std::string formatRadarPoseText() const;
  std::string formatRadarResultSummary() const;
  std::string formatRadarTransformText() const;

  NavigationNodeContext & context_;
  NavigationRuntime & runtime_;
  NavigationPointsWorkflow & points_workflow_;
  rclcpp::Node & node_;
  rclcpp::Logger logger_;
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_UI_COORDINATOR_HPP_
