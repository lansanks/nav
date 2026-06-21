#ifndef NAVIGATION_UI_MAP_UI_RENDERER_HPP_
#define NAVIGATION_UI_MAP_UI_RENDERER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "ui/map_ui_types.hpp"

namespace navigation::ui
{

class MapUiRenderer
{
public:
  MapUiRenderer(int width, int height, int panel_width = 320);

  int panelWidth(const MapUiState & ui_state) const;
  int canvasWidth(const MapUiState & ui_state) const;
  MapUiHit hitTest(int pixel_x, int pixel_y, const MapUiState & ui_state) const;
  void draw(cv::Mat & canvas, const MapUiState & ui_state, std::size_t point_count) const;

private:
  struct UiButton
  {
    MapUiAction action{MapUiAction::None};
    cv::Rect rect;
    std::string label;
  };

  std::vector<UiButton> uiButtons(const MapUiState & ui_state, bool navigation_active = false) const;
  cv::Rect togglePanelRect(const MapUiState & ui_state) const;
  static MapUiAction dropdownAnchorAction(MapDropdownMode mode);
  cv::Rect dropdownAnchorRect(MapDropdownMode mode, const MapUiState & ui_state) const;
  std::vector<cv::Rect> dropdownItemRects(const MapUiState & ui_state) const;
  cv::Rect paramsPopupRect(const MapUiState & ui_state) const;
  cv::Rect textInputPopupRect(const MapUiState & ui_state) const;
  cv::Rect textInputCloseButtonRect(const MapUiState & ui_state) const;
  cv::Rect settingsPopupRect(const MapUiState & ui_state) const;
  cv::Rect settingsCloseButtonRect(const MapUiState & ui_state) const;
  cv::Rect settingsApplyButtonRect(const MapUiState & ui_state) const;
  std::vector<cv::Rect> settingsFieldRects(const MapUiState & ui_state) const;
  std::vector<cv::Rect> paramRowRects(const MapUiState & ui_state) const;
  cv::Rect paramsSaveButtonRect(const MapUiState & ui_state) const;
  cv::Rect paramsLoadButtonRect(const MapUiState & ui_state) const;
  cv::Rect paramsWindowCloseButtonRect(const MapUiState & ui_state) const;
  int hitTestParamRow(int pixel_x, int pixel_y, const MapUiState & ui_state) const;
  int hitTestSettingsField(int pixel_x, int pixel_y, const MapUiState & ui_state) const;
  cv::Rect radarPopupRect(const MapUiState & ui_state) const;
  cv::Rect radarListenButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarSavePointButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarDataFileButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarPointsFileButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarRegisterButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarCloseButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarWindowCloseButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarAcceptButtonRect(const MapUiState & ui_state) const;
  cv::Rect radarRejectButtonRect(const MapUiState & ui_state) const;
  std::vector<cv::Rect> radarDropdownItemRects(const MapUiState & ui_state) const;
  cv::Rect fullscreenToggleRect() const;
  cv::Rect themeToggleRect() const;
  cv::Rect missionPlanToggleRect() const;

  static std::string shortenMiddle(const std::string & text, std::size_t max_len);
  static void putPanelText(
    cv::Mat & canvas,
    const std::string & text,
    cv::Point origin,
    double scale = 0.48,
    cv::Scalar color = cv::Scalar(226, 232, 238));

  void drawUiPanel(cv::Mat & canvas, const MapUiState & ui_state, std::size_t point_count) const;
  void drawPanelToggle(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawFullscreenToggle(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawThemeToggle(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawMissionPlanToggle(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawDropdownMenu(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawTextInputPopup(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawSettingsPopup(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawParamsPopup(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawRadarPopup(cv::Mat & canvas, const MapUiState & ui_state) const;
  void drawRadarButton(
    cv::Mat & canvas,
    const MapUiState & ui_state,
    const cv::Rect & rect,
    const std::string & label) const;
  void drawRadarDropdown(cv::Mat & canvas, const MapUiState & ui_state) const;

  int width_;
  int height_;
  int panel_width_;
};

}  // namespace navigation::ui

#endif  // NAVIGATION_UI_MAP_UI_RENDERER_HPP_
