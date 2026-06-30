#include "ui/map_ui_renderer.hpp"

#include <algorithm>
#include <iterator>

#include "opencv2/imgproc.hpp"

namespace navigation::ui
{
namespace
{

struct UiPalette
{
  cv::Scalar map_background;
  cv::Scalar grid;
  cv::Scalar grid_axis;
  cv::Scalar panel;
  cv::Scalar border;
  cv::Scalar surface;
  cv::Scalar surface_alt;
  cv::Scalar button;
  cv::Scalar button_border;
  cv::Scalar selected;
  cv::Scalar text;
  cv::Scalar text_muted;
  cv::Scalar title;
  cv::Scalar overlay;
};

UiPalette paletteFor(bool light_theme)
{
  if (light_theme) {
    return {
      cv::Scalar(245, 245, 245),
      cv::Scalar(222, 222, 222),
      cv::Scalar(178, 178, 178),
      cv::Scalar(250, 250, 250),
      cv::Scalar(198, 198, 198),
      cv::Scalar(255, 255, 255),
      cv::Scalar(238, 238, 238),
      cv::Scalar(236, 236, 236),
      cv::Scalar(160, 160, 160),
      cv::Scalar(218, 218, 218),
      cv::Scalar(28, 28, 28),
      cv::Scalar(92, 92, 92),
      cv::Scalar(12, 12, 12),
      cv::Scalar(255, 255, 255),
    };
  }

  return {
    cv::Scalar(18, 18, 18),
    cv::Scalar(40, 40, 40),
    cv::Scalar(76, 76, 76),
    cv::Scalar(24, 24, 24),
    cv::Scalar(64, 64, 64),
    cv::Scalar(28, 28, 28),
    cv::Scalar(38, 38, 38),
    cv::Scalar(45, 45, 45),
    cv::Scalar(105, 105, 105),
    cv::Scalar(72, 72, 72),
    cv::Scalar(232, 232, 232),
    cv::Scalar(182, 182, 182),
    cv::Scalar(250, 250, 250),
    cv::Scalar(0, 0, 0),
  };
}

std::string fitTextToWidth(
  const std::string & text,
  int max_width,
  double scale,
  int thickness = 1)
{
  if (max_width <= 0 || text.empty()) {
    return {};
  }

  int baseline = 0;
  const auto text_size = cv::getTextSize(
    text,
    cv::FONT_HERSHEY_SIMPLEX,
    scale,
    thickness,
    &baseline);
  if (text_size.width <= max_width) {
    return text;
  }

  constexpr char ellipsis[] = "...";
  const auto ellipsis_width = cv::getTextSize(
    ellipsis,
    cv::FONT_HERSHEY_SIMPLEX,
    scale,
    thickness,
    &baseline).width;
  if (ellipsis_width >= max_width) {
    return ".";
  }

  std::size_t left = 0;
  std::size_t right = text.size();
  while (left < right) {
    const std::size_t mid = left + (right - left + 1) / 2;
    const std::string candidate = text.substr(0, mid) + ellipsis;
    const auto candidate_width = cv::getTextSize(
      candidate,
      cv::FONT_HERSHEY_SIMPLEX,
      scale,
      thickness,
      &baseline).width;
    if (candidate_width <= max_width) {
      left = mid;
    } else {
      right = mid - 1;
    }
  }
  return text.substr(0, left) + ellipsis;
}

std::string planToggleLabel(MapPlanDisplayMode mode)
{
  if (mode == MapPlanDisplayMode::Full) {
    return "Plan Full";
  }
  if (mode == MapPlanDisplayMode::OrderOnly) {
    return "Plan No.";
  }
  return "Plan Off";
}

}  // namespace

MapUiRenderer::MapUiRenderer(int width, int height, int panel_width)
: width_(width),
  height_(height),
  panel_width_(panel_width)
{
}

int MapUiRenderer::panelWidth(const MapUiState & ui_state) const
{
  return ui_state.panel_collapsed ? 0 : panel_width_;
}

int MapUiRenderer::canvasWidth(const MapUiState & ui_state) const
{
  (void)ui_state;
  return width_ + panel_width_;
}

MapUiHit MapUiRenderer::hitTest(int pixel_x, int pixel_y, const MapUiState & ui_state) const
{
  if (ui_state.input_active) {
    const cv::Point point(pixel_x, pixel_y);
    if (textInputCloseButtonRect(ui_state).contains(point) ||
      !textInputPopupRect(ui_state).contains(point))
    {
      return {MapUiAction::InputClose, -1};
    }
    return {MapUiAction::UiOnly, -1};
  }

  if (fullscreenToggleRect().contains(cv::Point(pixel_x, pixel_y))) {
    return {MapUiAction::ToggleFullscreen, -1};
  }

  if (themeToggleRect().contains(cv::Point(pixel_x, pixel_y))) {
    return {MapUiAction::ToggleTheme, -1};
  }

  if (missionPlanToggleRect().contains(cv::Point(pixel_x, pixel_y))) {
    return {MapUiAction::ToggleMissionPlan, -1};
  }

  if (ui_state.radar_active) {
    if (radarWindowCloseButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarClose, -1};
    }

    if (ui_state.dropdown_mode == MapDropdownMode::RadarDataFile ||
      ui_state.dropdown_mode == MapDropdownMode::RadarPointsFile)
    {
      const auto item_rects = radarDropdownItemRects(ui_state);
      for (std::size_t i = 0; i < item_rects.size(); ++i) {
        if (item_rects[i].contains(cv::Point(pixel_x, pixel_y))) {
          return {MapUiAction::DropdownOption, static_cast<int>(i)};
        }
      }
    }

    if (ui_state.radar_confirm_active) {
      if (radarAcceptButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
        return {MapUiAction::RadarAcceptCalibration, -1};
      }
      if (radarRejectButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
        return {MapUiAction::RadarRejectCalibration, -1};
      }
      if (radarCloseButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
        return {MapUiAction::RadarClose, -1};
      }
      return {MapUiAction::UiOnly, -1};
    }

    if (radarListenButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarListen, -1};
    }
    if (radarSavePointButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarSavePoint, -1};
    }
    if (radarDataFileButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarSelectDataFile, -1};
    }
    if (radarPointsFileButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarSelectPointsFile, -1};
    }
    if (radarRegisterButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarRegister, -1};
    }
    if (radarCloseButtonRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::RadarClose, -1};
    }
    return {MapUiAction::UiOnly, -1};
  }

  if (ui_state.settings_active) {
    const cv::Point point(pixel_x, pixel_y);
    if (settingsCloseButtonRect(ui_state).contains(point) ||
      !settingsPopupRect(ui_state).contains(point))
    {
      return {MapUiAction::SettingsClose, -1};
    }
    if (settingsApplyButtonRect(ui_state).contains(point)) {
      return {MapUiAction::SettingsApply, -1};
    }
    const int field_index = hitTestSettingsField(pixel_x, pixel_y, ui_state);
    if (field_index >= 0) {
      return {MapUiAction::SettingsField, field_index};
    }
    return {MapUiAction::UiOnly, -1};
  }

  if (ui_state.segment_speed_active) {
    const cv::Point point(pixel_x, pixel_y);
    if (segmentSpeedCloseButtonRect(ui_state).contains(point) ||
      !segmentSpeedPopupRect(ui_state).contains(point))
    {
      return {MapUiAction::SegmentSpeedClose, -1};
    }
    if (segmentSpeedApplyButtonRect(ui_state).contains(point)) {
      return {MapUiAction::SegmentSpeedApply, -1};
    }
    if (segmentSpeedClearButtonRect(ui_state).contains(point)) {
      return {MapUiAction::SegmentSpeedClear, -1};
    }
    const int field_index = hitTestSegmentSpeedField(pixel_x, pixel_y, ui_state);
    if (field_index >= 0) {
      return {MapUiAction::SegmentSpeedField, field_index};
    }
    return {MapUiAction::UiOnly, -1};
  }

  if (ui_state.params_active) {
    const auto save_rect = paramsSaveButtonRect(ui_state);
    if (save_rect.contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::ParamSave, -1};
    }
    const auto load_rect = paramsLoadButtonRect(ui_state);
    if (load_rect.contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::ParamLoad, -1};
    }
    const auto close_rect = paramsWindowCloseButtonRect(ui_state);
    if (close_rect.contains(cv::Point(pixel_x, pixel_y))) {
      return {MapUiAction::ParamClose, -1};
    }
    const int param_index = hitTestParamRow(pixel_x, pixel_y, ui_state);
    if (param_index >= 0) {
      return {MapUiAction::ParamOption, param_index};
    }
    return {MapUiAction::UiOnly, -1};
  }

  if (togglePanelRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
    return {MapUiAction::TogglePanel, -1};
  }

  if (pixel_x >= width_) {
    if (ui_state.panel_collapsed) {
      return {MapUiAction::UiOnly, -1};
    }

    if (ui_state.dropdown_mode != MapDropdownMode::None) {
      const auto item_rects = dropdownItemRects(ui_state);
      for (std::size_t i = 0; i < item_rects.size(); ++i) {
        if (item_rects[i].contains(cv::Point(pixel_x, pixel_y))) {
          return {MapUiAction::DropdownOption, static_cast<int>(i)};
        }
      }
    }

    for (const auto & button : uiButtons(ui_state)) {
      if (button.rect.contains(cv::Point(pixel_x, pixel_y))) {
        return {button.action, -1};
      }
    }

    return {MapUiAction::UiOnly, -1};
  }

  if (!ui_state.panel_collapsed && ui_state.dropdown_mode != MapDropdownMode::None) {
    const auto item_rects = dropdownItemRects(ui_state);
    for (std::size_t i = 0; i < item_rects.size(); ++i) {
      if (item_rects[i].contains(cv::Point(pixel_x, pixel_y))) {
        return {MapUiAction::DropdownOption, static_cast<int>(i)};
      }
    }
  }

  for (const auto & button : uiButtons(ui_state)) {
    if (button.rect.contains(cv::Point(pixel_x, pixel_y))) {
      return {button.action, -1};
    }
  }
  return {};
}

void MapUiRenderer::draw(cv::Mat & canvas, const MapUiState & ui_state, std::size_t point_count) const
{
  drawUiPanel(canvas, ui_state, point_count);
  if (ui_state.input_active) {
    drawTextInputPopup(canvas, ui_state);
  }
}

std::vector<MapUiRenderer::UiButton> MapUiRenderer::uiButtons(
  const MapUiState & ui_state,
  bool navigation_active) const
{
  const int left = width_ + 22;
  int top = 104 - std::max(0, ui_state.panel_scroll_px);
  const int button_width = panel_width_ - 44;
  const int button_height = 36;
  const int gap = 12;
  std::vector<UiButton> buttons;
  buttons.reserve(9);

  auto add_button = [&](MapUiAction action, const std::string & label) {
    buttons.push_back({action, cv::Rect(left, top, button_width, button_height), label});
    top += button_height + gap;
  };

  add_button(MapUiAction::LoadPoints, "Load Points");
  add_button(MapUiAction::NewPoints, "New Points");
  add_button(MapUiAction::ChooseMap, "Choose Map");
  add_button(MapUiAction::SavePointsAs, "Save Points As");
  add_button(
    MapUiAction::ToggleRaceLogic,
    ui_state.race_logic == "mission" ? "Race: Mission" : "Race: Obstacle");
  add_button(MapUiAction::StartNavigation, navigation_active ? "Stop Navigation" : "Start Navigation");
  add_button(MapUiAction::Settings, "Task Settings");
  add_button(MapUiAction::OnlineParams, "Online Params");
  add_button(MapUiAction::Radar, "Radar");
  return buttons;
}

cv::Rect MapUiRenderer::togglePanelRect(const MapUiState & ui_state) const
{
  if (ui_state.panel_collapsed) {
    return cv::Rect(canvasWidth(ui_state) - 44, 14, 28, 28);
  }
  return cv::Rect(width_ + panel_width_ - 44, 14, 28, 28);
}

MapUiAction MapUiRenderer::dropdownAnchorAction(MapDropdownMode mode)
{
  if (mode == MapDropdownMode::LoadPoints) {
    return MapUiAction::LoadPoints;
  }
  if (mode == MapDropdownMode::ChooseMap) {
    return MapUiAction::ChooseMap;
  }
  if (mode == MapDropdownMode::ChooseController) {
    return MapUiAction::StartNavigation;
  }
  if (mode == MapDropdownMode::LoadParams) {
    return MapUiAction::OnlineParams;
  }
  if (mode == MapDropdownMode::RadarDataFile || mode == MapDropdownMode::RadarPointsFile) {
    return MapUiAction::Radar;
  }
  return MapUiAction::None;
}

cv::Rect MapUiRenderer::dropdownAnchorRect(MapDropdownMode mode, const MapUiState & ui_state) const
{
  const auto action = dropdownAnchorAction(mode);
  for (const auto & button : uiButtons(ui_state)) {
    if (button.action == action) {
      return button.rect;
    }
  }
  return {};
}

std::vector<cv::Rect> MapUiRenderer::dropdownItemRects(const MapUiState & ui_state) const
{
  std::vector<cv::Rect> rects;
  if (ui_state.dropdown_mode == MapDropdownMode::None || ui_state.dropdown_items.empty()) {
    return rects;
  }

  const auto anchor = dropdownAnchorRect(ui_state.dropdown_mode, ui_state);
  if (anchor.empty()) {
    return rects;
  }

  constexpr int item_height = 34;
  constexpr int menu_gap = 4;
  const int menu_top = anchor.y + anchor.height + menu_gap;
  const int max_visible_items = std::max(1, (height_ - menu_top - 18) / item_height);
  const int visible_items = std::min(static_cast<int>(ui_state.dropdown_items.size()), max_visible_items);
  rects.reserve(static_cast<std::size_t>(visible_items));

  for (int i = 0; i < visible_items; ++i) {
    rects.emplace_back(anchor.x, menu_top + i * item_height, anchor.width, item_height);
  }
  return rects;
}

cv::Rect MapUiRenderer::paramsPopupRect(const MapUiState & ui_state) const
{
  const int canvas_width = canvasWidth(ui_state);
  const int popup_width = 640;
  const int popup_height = 560;
  const int popup_x = (canvas_width - popup_width) / 2;
  const int popup_y = std::max(20, (height_ - popup_height) / 2);
  return cv::Rect(popup_x, popup_y, popup_width, popup_height);
}

cv::Rect MapUiRenderer::textInputPopupRect(const MapUiState & ui_state) const
{
  const int canvas_width = canvasWidth(ui_state);
  const int popup_width = 500;
  const int popup_height = 154;
  const int popup_x = (canvas_width - popup_width) / 2;
  const int popup_y = (height_ - popup_height) / 2;
  return cv::Rect(popup_x, popup_y, popup_width, popup_height);
}

cv::Rect MapUiRenderer::textInputCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = textInputPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 42, popup.y + 12, 26, 26);
}

cv::Rect MapUiRenderer::settingsPopupRect(const MapUiState & ui_state) const
{
  const int canvas_width = canvasWidth(ui_state);
  const int popup_width = 680;
  const int popup_height = 700;
  const int popup_x = (canvas_width - popup_width) / 2;
  const int popup_y = std::max(18, (height_ - popup_height) / 2);
  return cv::Rect(popup_x, popup_y, popup_width, popup_height);
}

cv::Rect MapUiRenderer::settingsCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = settingsPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 42, popup.y + 12, 26, 26);
}

cv::Rect MapUiRenderer::settingsApplyButtonRect(const MapUiState & ui_state) const
{
  const auto popup = settingsPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 150, popup.y + popup.height - 70, 104, 36);
}

std::vector<cv::Rect> MapUiRenderer::settingsFieldRects(const MapUiState & ui_state) const
{
  std::vector<cv::Rect> rects;
  const auto popup = settingsPopupRect(ui_state);
  constexpr int row_height = 42;
  int y = popup.y + 80;
  rects.reserve(ui_state.settings_field_names.size());
  for (std::size_t i = 0; i < ui_state.settings_field_names.size(); ++i) {
    rects.emplace_back(popup.x + 24, y, popup.width - 48, row_height - 6);
    y += row_height;
  }
  return rects;
}

cv::Rect MapUiRenderer::segmentSpeedPopupRect(const MapUiState & ui_state) const
{
  const int canvas_width = canvasWidth(ui_state);
  const int popup_width = 620;
  const int popup_height = std::min(430, std::max(360, height_ - 24));
  const int popup_x = (canvas_width - popup_width) / 2;
  const int popup_y = std::max(12, (height_ - popup_height) / 2);
  return cv::Rect(popup_x, popup_y, popup_width, popup_height);
}

cv::Rect MapUiRenderer::segmentSpeedCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = segmentSpeedPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 42, popup.y + 12, 26, 26);
}

cv::Rect MapUiRenderer::segmentSpeedApplyButtonRect(const MapUiState & ui_state) const
{
  const auto popup = segmentSpeedPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 274, popup.y + popup.height - 58, 104, 34);
}

cv::Rect MapUiRenderer::segmentSpeedClearButtonRect(const MapUiState & ui_state) const
{
  const auto popup = segmentSpeedPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 146, popup.y + popup.height - 58, 100, 34);
}

std::vector<cv::Rect> MapUiRenderer::segmentSpeedFieldRects(const MapUiState & ui_state) const
{
  std::vector<cv::Rect> rects;
  const auto popup = segmentSpeedPopupRect(ui_state);
  constexpr int row_height = 35;
  int y = popup.y + 76;
  rects.reserve(ui_state.segment_speed_field_names.size());
  for (std::size_t i = 0; i < ui_state.segment_speed_field_names.size(); ++i) {
    rects.emplace_back(popup.x + 24, y, popup.width - 48, row_height - 4);
    y += row_height;
  }
  return rects;
}

std::vector<cv::Rect> MapUiRenderer::paramRowRects(const MapUiState & ui_state) const
{
  std::vector<cv::Rect> rects;
  const auto popup = paramsPopupRect(ui_state);
  constexpr int row_height = 34;
  int y = popup.y + 74;
  rects.reserve(ui_state.param_names.size());
  for (std::size_t i = 0; i < ui_state.param_names.size(); ++i) {
    rects.emplace_back(popup.x + 18, y - 23, popup.width - 36, row_height);
    y += row_height;
  }
  return rects;
}

cv::Rect MapUiRenderer::paramsSaveButtonRect(const MapUiState & ui_state) const
{
  const auto popup = paramsPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 244, popup.y + popup.height - 78, 96, 34);
}

cv::Rect MapUiRenderer::paramsLoadButtonRect(const MapUiState & ui_state) const
{
  const auto popup = paramsPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 132, popup.y + popup.height - 78, 96, 34);
}

cv::Rect MapUiRenderer::paramsWindowCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = paramsPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 42, popup.y + 12, 26, 26);
}

cv::Rect MapUiRenderer::fullscreenToggleRect() const
{
  return cv::Rect(12, 12, 112, 34);
}

cv::Rect MapUiRenderer::themeToggleRect() const
{
  return cv::Rect(136, 12, 112, 34);
}

cv::Rect MapUiRenderer::missionPlanToggleRect() const
{
  return cv::Rect(260, 12, 128, 34);
}

cv::Rect MapUiRenderer::radarPopupRect(const MapUiState & ui_state) const
{
  const int canvas_width = canvasWidth(ui_state);
  const int popup_width = 680;
  const int popup_height = 560;
  const int popup_x = (canvas_width - popup_width) / 2;
  const int popup_y = std::max(20, (height_ - popup_height) / 2);
  return cv::Rect(popup_x, popup_y, popup_width, popup_height);
}

cv::Rect MapUiRenderer::radarListenButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + 24, popup.y + 114, 132, 36);
}

cv::Rect MapUiRenderer::radarSavePointButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + 172, popup.y + 114, 132, 36);
}

cv::Rect MapUiRenderer::radarDataFileButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + 24, popup.y + 262, 292, 36);
}

cv::Rect MapUiRenderer::radarPointsFileButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + 340, popup.y + 262, 292, 36);
}

cv::Rect MapUiRenderer::radarRegisterButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 300, popup.y + popup.height - 72, 124, 36);
}

cv::Rect MapUiRenderer::radarCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 152, popup.y + popup.height - 72, 104, 36);
}

cv::Rect MapUiRenderer::radarWindowCloseButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 42, popup.y + 12, 26, 26);
}

cv::Rect MapUiRenderer::radarAcceptButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 300, popup.y + popup.height - 72, 124, 36);
}

cv::Rect MapUiRenderer::radarRejectButtonRect(const MapUiState & ui_state) const
{
  const auto popup = radarPopupRect(ui_state);
  return cv::Rect(popup.x + popup.width - 152, popup.y + popup.height - 72, 104, 36);
}

std::vector<cv::Rect> MapUiRenderer::radarDropdownItemRects(const MapUiState & ui_state) const
{
  std::vector<cv::Rect> rects;
  if (ui_state.dropdown_mode != MapDropdownMode::RadarDataFile &&
    ui_state.dropdown_mode != MapDropdownMode::RadarPointsFile)
  {
    return rects;
  }

  const auto anchor = ui_state.dropdown_mode == MapDropdownMode::RadarDataFile ?
    radarDataFileButtonRect(ui_state) :
    radarPointsFileButtonRect(ui_state);
  constexpr int item_height = 34;
  constexpr int max_visible_items = 6;
  const int visible_items = std::min(static_cast<int>(ui_state.dropdown_items.size()), max_visible_items);
  rects.reserve(static_cast<std::size_t>(visible_items));
  for (int i = 0; i < visible_items; ++i) {
    rects.emplace_back(anchor.x, anchor.y + anchor.height + 4 + i * item_height, anchor.width, item_height);
  }
  return rects;
}

int MapUiRenderer::hitTestParamRow(int pixel_x, int pixel_y, const MapUiState & ui_state) const
{
  if (!paramsPopupRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
    return -1;
  }

  const auto rects = paramRowRects(ui_state);
  for (std::size_t i = 0; i < rects.size(); ++i) {
    if (rects[i].contains(cv::Point(pixel_x, pixel_y))) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int MapUiRenderer::hitTestSettingsField(int pixel_x, int pixel_y, const MapUiState & ui_state) const
{
  if (!settingsPopupRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
    return -1;
  }

  const auto rects = settingsFieldRects(ui_state);
  for (std::size_t i = 0; i < rects.size(); ++i) {
    if (rects[i].contains(cv::Point(pixel_x, pixel_y))) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int MapUiRenderer::hitTestSegmentSpeedField(int pixel_x, int pixel_y, const MapUiState & ui_state) const
{
  if (!segmentSpeedPopupRect(ui_state).contains(cv::Point(pixel_x, pixel_y))) {
    return -1;
  }

  const auto rects = segmentSpeedFieldRects(ui_state);
  for (std::size_t i = 0; i < rects.size(); ++i) {
    if (rects[i].contains(cv::Point(pixel_x, pixel_y))) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string MapUiRenderer::shortenMiddle(const std::string & text, std::size_t max_len)
{
  if (text.size() <= max_len) {
    return text;
  }
  if (max_len <= 5) {
    return text.substr(0, max_len);
  }
  const std::size_t head = (max_len - 3) / 2;
  const std::size_t tail = max_len - 3 - head;
  return text.substr(0, head) + "..." + text.substr(text.size() - tail);
}

void MapUiRenderer::putPanelText(
  cv::Mat & canvas,
  const std::string & text,
  cv::Point origin,
  double scale,
  cv::Scalar color)
{
  cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

void MapUiRenderer::drawUiPanel(cv::Mat & canvas, const MapUiState & ui_state, std::size_t point_count) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int panel_left = width_;
  const int active_panel_width = panelWidth(ui_state);
  drawFullscreenToggle(canvas, ui_state);
  drawThemeToggle(canvas, ui_state);
  drawMissionPlanToggle(canvas, ui_state);

  if (!ui_state.message.empty()) {
    constexpr int margin = 12;
    constexpr int padding_x = 10;
    constexpr int message_height = 36;
    const int max_text_width = std::max(40, std::min(width_ - margin * 2 - padding_x * 2, 420));
    const std::string message_text = fitTextToWidth(ui_state.message, max_text_width, 0.43);
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      message_text,
      cv::FONT_HERSHEY_SIMPLEX,
      0.43,
      1,
      &baseline);
    const int message_width = std::min(width_ - margin * 2, text_size.width + padding_x * 2);
    const cv::Rect message_rect(
      std::max(margin, width_ - message_width - margin),
      std::max(margin, height_ - message_height - margin),
      message_width,
      message_height);
    cv::rectangle(canvas, message_rect, palette.surface_alt, cv::FILLED);
    cv::rectangle(canvas, message_rect, palette.button_border, 1, cv::LINE_AA);
    const bool disconnected_message =
      !ui_state.core_connected && ui_state.message.find("DISCONNECTED") != std::string::npos;
    const auto msg_color = disconnected_message ? cv::Scalar(0, 0, 255) : palette.text;
    putPanelText(
      canvas,
      message_text,
      cv::Point(message_rect.x + padding_x, message_rect.y + 23),
      0.43,
      msg_color);
  }

  if (ui_state.panel_collapsed) {
    drawPanelToggle(canvas, ui_state);
    return;
  }

  cv::rectangle(canvas, cv::Rect(panel_left, 0, active_panel_width, height_), palette.panel, cv::FILLED);
  cv::line(canvas, cv::Point(panel_left, 0), cv::Point(panel_left, height_), palette.border, 1, cv::LINE_AA);
  drawPanelToggle(canvas, ui_state);

  cv::putText(canvas, "Map Controls", cv::Point(panel_left + 22, 38), cv::FONT_HERSHEY_SIMPLEX, 0.72, palette.title, 2, cv::LINE_AA);
  putPanelText(canvas, "Left click map: add point", cv::Point(panel_left + 22, 66), 0.48, palette.text);
  putPanelText(canvas, "Wheel: zoom, middle drag: pan", cv::Point(panel_left + 22, 86), 0.43, palette.text_muted);

  const auto buttons = uiButtons(ui_state, ui_state.navigation_active);
  for (const auto & button : buttons) {
    cv::rectangle(canvas, button.rect, palette.button, cv::FILLED);
    cv::rectangle(canvas, button.rect, palette.button_border, 1, cv::LINE_AA);
    cv::putText(canvas, button.label, cv::Point(button.rect.x + 14, button.rect.y + 24), cv::FONT_HERSHEY_SIMPLEX, 0.52, palette.text, 1, cv::LINE_AA);
  }

  const int scroll_y = std::max(0, ui_state.panel_scroll_px);
  int y = buttons.empty() ? 516 - scroll_y : buttons.back().rect.y + buttons.back().rect.height + 28;
  putPanelText(canvas, "Points: " + std::to_string(point_count), cv::Point(panel_left + 22, y), 0.48, palette.text);
  putPanelText(canvas, "Right click: remove last", cv::Point(panel_left + 22, y + 24), 0.43, palette.text_muted);
  putPanelText(canvas, "Press C: clear", cv::Point(panel_left + 22, y + 50), 0.43, palette.text_muted);
  putPanelText(canvas, "Esc/Q: quit", cv::Point(panel_left + 22, y + 76), 0.43, palette.text_muted);

  y += 108;
  putPanelText(canvas, "Race:", cv::Point(panel_left + 22, y), 0.48, palette.text);
  putPanelText(
    canvas,
    ui_state.race_logic == "mission" ? "Mission" : "Obstacle",
    cv::Point(panel_left + 22, y + 28),
    0.43,
    palette.text_muted);

  y += 70;
  putPanelText(canvas, "Controller:", cv::Point(panel_left + 22, y), 0.48, palette.text);
  const std::string controller_name = ui_state.controller_name.empty() ? std::string("None") : ui_state.controller_name;
  putPanelText(canvas, shortenMiddle(controller_name, 34), cv::Point(panel_left + 22, y + 28), 0.43, palette.text_muted);

  y += 70;
  const std::string navigation_status = ui_state.navigation_status.empty() ?
    std::string(ui_state.navigation_active ? "Running" : "Stopped") :
    ui_state.navigation_status;
  putPanelText(canvas, "Navigation:", cv::Point(panel_left + 22, y), 0.48, palette.text);
  putPanelText(canvas, shortenMiddle(navigation_status, 34), cv::Point(panel_left + 22, y + 28), 0.43, palette.text_muted);
  if (ui_state.navigation_point_count > 0) {
    const std::string progress = "Target: " +
      std::to_string(std::min(ui_state.navigation_target_index + 1, ui_state.navigation_point_count)) +
      "/" + std::to_string(ui_state.navigation_point_count);
    putPanelText(canvas, progress, cv::Point(panel_left + 22, y + 54), 0.43, palette.text_muted);
  }

  y += 84;
  putPanelText(canvas, "Core:", cv::Point(panel_left + 22, y), 0.48, palette.text);
  const std::string core_status = ui_state.core_connected ? "Connected" : "DISCONNECTED";
  const auto core_color = ui_state.core_connected ? palette.text_muted : cv::Scalar(64, 128, 255);
  putPanelText(canvas, core_status, cv::Point(panel_left + 22, y + 28), 0.43, core_color);

  const int content_height = 900;
  const int max_scroll = std::max(0, content_height - height_);
  if (max_scroll > 0) {
    const int track_x = panel_left + active_panel_width - 8;
    const int track_y = 62;
    const int track_h = std::max(32, height_ - 86);
    const int thumb_h = std::max(26, track_h * height_ / content_height);
    const int thumb_y = track_y + (track_h - thumb_h) * std::min(scroll_y, max_scroll) / max_scroll;
    cv::line(canvas, cv::Point(track_x, track_y), cv::Point(track_x, track_y + track_h), palette.border, 2, cv::LINE_AA);
    cv::rectangle(canvas, cv::Rect(track_x - 3, thumb_y, 6, thumb_h), palette.text_muted, cv::FILLED);
  }

  drawDropdownMenu(canvas, ui_state);
  if (ui_state.settings_active) {
    drawSettingsPopup(canvas, ui_state);
  }
  if (ui_state.segment_speed_active) {
    drawSegmentSpeedPopup(canvas, ui_state);
  }
  if (ui_state.params_active) {
    drawParamsPopup(canvas, ui_state);
  }
  if (ui_state.radar_active) {
    drawRadarPopup(canvas, ui_state);
  }
}

void MapUiRenderer::drawPanelToggle(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const auto rect = togglePanelRect(ui_state);
  cv::rectangle(canvas, rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);

  const std::string label = ui_state.panel_collapsed ? "<" : ">";
  cv::putText(
    canvas,
    label,
    cv::Point(rect.x + 9, rect.y + 20),
    cv::FONT_HERSHEY_SIMPLEX,
    0.62,
    palette.text,
    2,
    cv::LINE_AA);
}

void MapUiRenderer::drawThemeToggle(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const auto rect = themeToggleRect();
  cv::rectangle(canvas, rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(
    canvas,
    ui_state.light_theme ? "Light" : "Dark",
    cv::Point(rect.x + 16, rect.y + 23),
    0.50,
    palette.text);
}

void MapUiRenderer::drawFullscreenToggle(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const auto rect = fullscreenToggleRect();
  cv::rectangle(canvas, rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(
    canvas,
    ui_state.fullscreen ? "Window" : "Full",
    cv::Point(rect.x + 16, rect.y + 23),
    0.50,
    palette.text);
}

void MapUiRenderer::drawMissionPlanToggle(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const auto rect = missionPlanToggleRect();
  cv::rectangle(canvas, rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(
    canvas,
    planToggleLabel(ui_state.mission_plan_display_mode),
    cv::Point(rect.x + 16, rect.y + 23),
    0.50,
    palette.text);
}

void MapUiRenderer::drawDropdownMenu(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  if (ui_state.dropdown_mode == MapDropdownMode::None) {
    return;
  }
  if (ui_state.dropdown_mode == MapDropdownMode::RadarDataFile ||
    ui_state.dropdown_mode == MapDropdownMode::RadarPointsFile)
  {
    return;
  }

  const auto anchor = dropdownAnchorRect(ui_state.dropdown_mode, ui_state);
  if (anchor.empty()) {
    return;
  }

  const auto item_rects = dropdownItemRects(ui_state);
  if (item_rects.empty()) {
    const cv::Rect empty_rect(anchor.x, anchor.y + anchor.height + 4, anchor.width, 38);
    cv::rectangle(canvas, empty_rect, palette.surface, cv::FILLED);
    cv::rectangle(canvas, empty_rect, palette.button_border, 1, cv::LINE_AA);
    const std::string empty_text = ui_state.dropdown_mode == MapDropdownMode::ChooseController ?
      "No controllers found" :
      (ui_state.dropdown_mode == MapDropdownMode::LoadParams ? "No params found" : "No files found");
    putPanelText(canvas, empty_text, cv::Point(empty_rect.x + 12, empty_rect.y + 24), 0.45, palette.text);
    return;
  }

  const cv::Rect menu_rect(
    item_rects.front().x,
    item_rects.front().y,
    item_rects.front().width,
    item_rects.back().y + item_rects.back().height - item_rects.front().y);
  cv::rectangle(canvas, menu_rect, palette.surface, cv::FILLED);
  cv::rectangle(canvas, menu_rect, palette.button_border, 1, cv::LINE_AA);

  for (std::size_t i = 0; i < item_rects.size(); ++i) {
    const auto & rect = item_rects[i];
    const int option_index = static_cast<int>(i);
    const auto marked_iter = std::find(
      ui_state.dropdown_marked_indices.begin(),
      ui_state.dropdown_marked_indices.end(),
      option_index);
    const bool marked = marked_iter != ui_state.dropdown_marked_indices.end();
    const bool selected = option_index == ui_state.dropdown_selected_index;
    if (marked || selected) {
      cv::rectangle(canvas, rect, palette.selected, cv::FILLED);
    }
    if (selected && marked) {
      cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);
    }
    cv::line(canvas, cv::Point(rect.x, rect.y + rect.height), cv::Point(rect.x + rect.width, rect.y + rect.height), palette.border, 1, cv::LINE_AA);
    std::string label = ui_state.dropdown_items[i];
    if (marked) {
      const auto order = std::distance(ui_state.dropdown_marked_indices.begin(), marked_iter) + 1;
      label = std::to_string(order) + ". " + label;
    }
    putPanelText(canvas, shortenMiddle(label, 31), cv::Point(rect.x + 12, rect.y + 23), 0.45, palette.text);
  }
}

void MapUiRenderer::drawTextInputPopup(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int canvas_width = canvasWidth(ui_state);
  cv::Mat overlay = canvas.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, canvas_width, height_), palette.overlay, cv::FILLED);
  cv::addWeighted(overlay, 0.46, canvas, 0.54, 0.0, canvas);

  const auto popup = textInputPopupRect(ui_state);
  const int popup_width = popup.width;
  const int popup_x = popup.x;
  const int popup_y = popup.y;
  cv::rectangle(canvas, popup, palette.surface, cv::FILLED);
  cv::rectangle(canvas, popup, palette.button_border, 1, cv::LINE_AA);

  const auto close_rect = textInputCloseButtonRect(ui_state);
  cv::rectangle(canvas, close_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, close_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "X", cv::Point(close_rect.x + 7, close_rect.y + 19), 0.50, palette.text);

  putPanelText(canvas, ui_state.input_label, cv::Point(popup_x + 24, popup_y + 36), 0.58, palette.text);
  const cv::Rect input_rect(popup_x + 24, popup_y + 58, popup_width - 48, 42);
  cv::rectangle(canvas, input_rect, palette.surface_alt, cv::FILLED);
  cv::rectangle(canvas, input_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, shortenMiddle(ui_state.input_text + "_", 44), cv::Point(input_rect.x + 12, input_rect.y + 27), 0.52, palette.text);
  putPanelText(canvas, "Enter confirm, Esc cancel", cv::Point(popup_x + 24, popup_y + 128), 0.44, palette.text_muted);
}

void MapUiRenderer::drawSettingsPopup(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int canvas_width = canvasWidth(ui_state);
  cv::Mat overlay = canvas.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, canvas_width, height_), palette.overlay, cv::FILLED);
  cv::addWeighted(overlay, 0.48, canvas, 0.52, 0.0, canvas);

  const auto popup = settingsPopupRect(ui_state);
  cv::rectangle(canvas, popup, palette.surface, cv::FILLED);
  cv::rectangle(canvas, popup, palette.button_border, 1, cv::LINE_AA);

  const auto close_rect = settingsCloseButtonRect(ui_state);
  cv::rectangle(canvas, close_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, close_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "X", cv::Point(close_rect.x + 7, close_rect.y + 19), 0.50, palette.text);

  putPanelText(canvas, "Task Settings", cv::Point(popup.x + 24, popup.y + 38), 0.68, palette.title);
  putPanelText(
    canvas,
    "Slot categories are slot 0..7 category ids: 0 food, 1 tool, 2 instrument, 3 medicine.",
    cv::Point(popup.x + 24, popup.y + 62),
    0.36,
    palette.text_muted);

  const int name_x = popup.x + 36;
  const int value_x = popup.x + 292;
  const auto field_rects = settingsFieldRects(ui_state);
  for (std::size_t i = 0; i < ui_state.settings_field_names.size(); ++i) {
    const auto & row = field_rects[i];
    if (static_cast<int>(i) == ui_state.settings_selected_index) {
      cv::rectangle(canvas, row, palette.selected, cv::FILLED);
    } else {
      cv::rectangle(canvas, row, palette.surface_alt, cv::FILLED);
    }
    cv::rectangle(canvas, row, palette.button_border, 1, cv::LINE_AA);

    const int text_y = row.y + 22;
    putPanelText(canvas, ui_state.settings_field_names[i], cv::Point(name_x, text_y), 0.44, palette.text);
    std::string value = i < ui_state.settings_field_values.size() ? ui_state.settings_field_values[i] : "";
    if (ui_state.settings_editing && static_cast<int>(i) == ui_state.settings_selected_index) {
      value = ui_state.settings_edit_text + "_";
    }
    putPanelText(
      canvas,
      fitTextToWidth(value, popup.x + popup.width - value_x - 36, 0.44),
      cv::Point(value_x, text_y),
      0.44,
      palette.text);
  }

  if (!ui_state.mission_plan_summary.empty()) {
    putPanelText(
      canvas,
      fitTextToWidth(ui_state.mission_plan_summary, popup.width - 48, 0.42),
      cv::Point(popup.x + 24, popup.y + popup.height - 100),
      0.42,
      palette.text_muted);
  }

  const auto apply_rect = settingsApplyButtonRect(ui_state);
  cv::rectangle(canvas, apply_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, apply_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "Apply", cv::Point(apply_rect.x + 27, apply_rect.y + 23), 0.48, palette.text);
  putPanelText(
    canvas,
    "Enter edit/apply field  Up/Down select  Esc close",
    cv::Point(popup.x + 24, popup.y + popup.height - 18),
    0.38,
    palette.text_muted);
}

void MapUiRenderer::drawSegmentSpeedPopup(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int canvas_width = canvasWidth(ui_state);
  cv::Mat overlay = canvas.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, canvas_width, height_), palette.overlay, cv::FILLED);
  cv::addWeighted(overlay, 0.48, canvas, 0.52, 0.0, canvas);

  const auto popup = segmentSpeedPopupRect(ui_state);
  cv::rectangle(canvas, popup, palette.surface, cv::FILLED);
  cv::rectangle(canvas, popup, palette.button_border, 1, cv::LINE_AA);

  const auto close_rect = segmentSpeedCloseButtonRect(ui_state);
  cv::rectangle(canvas, close_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, close_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "X", cv::Point(close_rect.x + 7, close_rect.y + 19), 0.50, palette.text);

  const std::string title = ui_state.segment_speed_title.empty() ?
    std::string("Segment Speed") :
    ui_state.segment_speed_title;
  putPanelText(canvas, title, cv::Point(popup.x + 24, popup.y + 38), 0.68, palette.title);
  putPanelText(
    canvas,
    "Mode switches between constant speed and P control; Level loads defaults.",
    cv::Point(popup.x + 24, popup.y + 62),
    0.38,
    palette.text_muted);

  const int name_x = popup.x + 36;
  const int value_x = popup.x + 322;
  const auto field_rects = segmentSpeedFieldRects(ui_state);
  for (std::size_t i = 0; i < ui_state.segment_speed_field_names.size(); ++i) {
    const auto & row = field_rects[i];
    if (static_cast<int>(i) == ui_state.segment_speed_selected_index) {
      cv::rectangle(canvas, row, palette.selected, cv::FILLED);
    } else {
      cv::rectangle(canvas, row, palette.surface_alt, cv::FILLED);
    }
    cv::rectangle(canvas, row, palette.button_border, 1, cv::LINE_AA);

    const int text_y = row.y + 24;
    putPanelText(canvas, ui_state.segment_speed_field_names[i], cv::Point(name_x, text_y), 0.45, palette.text);
    std::string value = i < ui_state.segment_speed_field_values.size() ?
      ui_state.segment_speed_field_values[i] :
      "";
    if (ui_state.segment_speed_editing && static_cast<int>(i) == ui_state.segment_speed_selected_index) {
      value = ui_state.segment_speed_edit_text + "_";
    }
    putPanelText(
      canvas,
      fitTextToWidth(value, popup.x + popup.width - value_x - 40, 0.45),
      cv::Point(value_x, text_y),
      0.45,
      palette.text);
  }

  const auto apply_rect = segmentSpeedApplyButtonRect(ui_state);
  const auto clear_rect = segmentSpeedClearButtonRect(ui_state);
  cv::rectangle(canvas, apply_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, apply_rect, palette.button_border, 1, cv::LINE_AA);
  cv::rectangle(canvas, clear_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, clear_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "Apply", cv::Point(apply_rect.x + 27, apply_rect.y + 23), 0.48, palette.text);
  putPanelText(canvas, "Clear", cv::Point(clear_rect.x + 27, clear_rect.y + 23), 0.48, palette.text);

  putPanelText(
    canvas,
    fitTextToWidth(
      "Enter edit/toggle  Double-click edit  Level 1..7  Esc close",
      apply_rect.x - popup.x - 48,
      0.38),
    cv::Point(popup.x + 24, popup.y + popup.height - 18),
    0.38,
    palette.text_muted);
}

void MapUiRenderer::drawParamsPopup(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int canvas_width = canvasWidth(ui_state);
  cv::Mat overlay = canvas.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, canvas_width, height_), palette.overlay, cv::FILLED);
  cv::addWeighted(overlay, 0.50, canvas, 0.50, 0.0, canvas);

  const auto popup = paramsPopupRect(ui_state);
  const int popup_height = popup.height;
  const int popup_x = popup.x;
  const int popup_y = popup.y;
  cv::rectangle(canvas, popup, palette.surface, cv::FILLED);
  cv::rectangle(canvas, popup, palette.button_border, 1, cv::LINE_AA);

  const auto close_rect = paramsWindowCloseButtonRect(ui_state);
  cv::rectangle(canvas, close_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, close_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "X", cv::Point(close_rect.x + 7, close_rect.y + 19), 0.50, palette.text);

  putPanelText(canvas, "Online Params", cv::Point(popup_x + 24, popup_y + 36), 0.68, palette.title);
  const int name_x = popup_x + 28;
  const int value_x = popup_x + 380;
  const auto row_rects = paramRowRects(ui_state);
  for (std::size_t i = 0; i < ui_state.param_names.size(); ++i) {
    const auto & row = row_rects[i];
    if (i == ui_state.param_selected_index) {
      cv::rectangle(canvas, row, palette.selected, cv::FILLED);
    }

    const int text_y = row.y + 23;
    putPanelText(canvas, ui_state.param_names[i], cv::Point(name_x, text_y), 0.45, palette.text);
    std::string value = i < ui_state.param_values.size() ? ui_state.param_values[i] : "";
    if (ui_state.params_editing && i == ui_state.param_selected_index) {
      value = ui_state.params_edit_text + "_";
    }
    putPanelText(canvas, value, cv::Point(value_x, text_y), 0.45, palette.text);
  }

  const auto save_rect = paramsSaveButtonRect(ui_state);
  const auto load_rect = paramsLoadButtonRect(ui_state);
  cv::rectangle(canvas, save_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, save_rect, palette.button_border, 1, cv::LINE_AA);
  cv::rectangle(canvas, load_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, load_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "Save", cv::Point(save_rect.x + 26, save_rect.y + 23), 0.48, palette.text);
  putPanelText(canvas, "Load", cv::Point(load_rect.x + 28, load_rect.y + 23), 0.48, palette.text);

  putPanelText(canvas, "Enter edit/apply  Up/Down select  Esc close", cv::Point(popup_x + 24, popup_y + popup_height - 54), 0.43, palette.text_muted);
  putPanelText(canvas, "Values apply immediately after Enter.", cv::Point(popup_x + 24, popup_y + popup_height - 28), 0.40, palette.text_muted);
}

void MapUiRenderer::drawRadarButton(
  cv::Mat & canvas,
  const MapUiState & ui_state,
  const cv::Rect & rect,
  const std::string & label) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  cv::rectangle(canvas, rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, shortenMiddle(label, 24), cv::Point(rect.x + 14, rect.y + 24), 0.48, palette.text);
}

void MapUiRenderer::drawRadarDropdown(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  if (ui_state.dropdown_mode != MapDropdownMode::RadarDataFile &&
    ui_state.dropdown_mode != MapDropdownMode::RadarPointsFile)
  {
    return;
  }

  const auto item_rects = radarDropdownItemRects(ui_state);
  const auto anchor = ui_state.dropdown_mode == MapDropdownMode::RadarDataFile ?
    radarDataFileButtonRect(ui_state) :
    radarPointsFileButtonRect(ui_state);
  if (item_rects.empty()) {
    const cv::Rect empty_rect(anchor.x, anchor.y + anchor.height + 4, anchor.width, 38);
    cv::rectangle(canvas, empty_rect, palette.surface, cv::FILLED);
    cv::rectangle(canvas, empty_rect, palette.button_border, 1, cv::LINE_AA);
    putPanelText(canvas, "No files found", cv::Point(empty_rect.x + 12, empty_rect.y + 24), 0.45, palette.text);
    return;
  }

  const cv::Rect menu_rect(
    item_rects.front().x,
    item_rects.front().y,
    item_rects.front().width,
    item_rects.back().y + item_rects.back().height - item_rects.front().y);
  cv::rectangle(canvas, menu_rect, palette.surface, cv::FILLED);
  cv::rectangle(canvas, menu_rect, palette.button_border, 1, cv::LINE_AA);

  for (std::size_t i = 0; i < item_rects.size(); ++i) {
    const auto & rect = item_rects[i];
    if (static_cast<int>(i) == ui_state.dropdown_selected_index) {
      cv::rectangle(canvas, rect, palette.selected, cv::FILLED);
    }
    cv::line(canvas, cv::Point(rect.x, rect.y + rect.height), cv::Point(rect.x + rect.width, rect.y + rect.height), palette.border, 1, cv::LINE_AA);
    putPanelText(canvas, shortenMiddle(ui_state.dropdown_items[i], 27), cv::Point(rect.x + 12, rect.y + 23), 0.43, palette.text);
  }
}

void MapUiRenderer::drawRadarPopup(cv::Mat & canvas, const MapUiState & ui_state) const
{
  const auto palette = paletteFor(ui_state.light_theme);
  const int canvas_width = canvasWidth(ui_state);
  cv::Mat overlay = canvas.clone();
  cv::rectangle(overlay, cv::Rect(0, 0, canvas_width, height_), palette.overlay, cv::FILLED);
  cv::addWeighted(overlay, 0.50, canvas, 0.50, 0.0, canvas);

  const auto popup = radarPopupRect(ui_state);
  cv::rectangle(canvas, popup, palette.surface, cv::FILLED);
  cv::rectangle(canvas, popup, palette.button_border, 1, cv::LINE_AA);

  const auto close_rect = radarWindowCloseButtonRect(ui_state);
  cv::rectangle(canvas, close_rect, palette.button, cv::FILLED);
  cv::rectangle(canvas, close_rect, palette.button_border, 1, cv::LINE_AA);
  putPanelText(canvas, "X", cv::Point(close_rect.x + 7, close_rect.y + 19), 0.50, palette.text);

  putPanelText(canvas, "Radar Calibration", cv::Point(popup.x + 24, popup.y + 38), 0.68, palette.title);
  putPanelText(canvas, "Topic: " + ui_state.radar_topic, cv::Point(popup.x + 24, popup.y + 68), 0.44, palette.text_muted);

  if (ui_state.radar_confirm_active) {
    putPanelText(canvas, "Registration Result", cv::Point(popup.x + 24, popup.y + 112), 0.58, palette.text);
    putPanelText(canvas, ui_state.radar_result_summary, cv::Point(popup.x + 24, popup.y + 148), 0.46, palette.text);
    putPanelText(canvas, ui_state.radar_transform_text, cv::Point(popup.x + 24, popup.y + 178), 0.43, palette.text_muted);
    if (ui_state.radar_result_unstable) {
      putPanelText(canvas, "Warning: points are nearly collinear or overlapping.", cv::Point(popup.x + 24, popup.y + 212), 0.43, palette.text_muted);
    }
    putPanelText(canvas, "Accept saves the calibration YAML.", cv::Point(popup.x + 24, popup.y + popup.height - 46), 0.42, palette.text_muted);
    drawRadarButton(canvas, ui_state, radarAcceptButtonRect(ui_state), "Accept");
    drawRadarButton(canvas, ui_state, radarRejectButtonRect(ui_state), "Reject");
    return;
  }

  drawRadarButton(canvas, ui_state, radarListenButtonRect(ui_state), ui_state.radar_listening ? "Stop" : "Listen");
  drawRadarButton(canvas, ui_state, radarSavePointButtonRect(ui_state), "Save Point");

  const std::string pose_status = ui_state.radar_pose_valid ? ui_state.radar_pose_text : "Pose: waiting";
  putPanelText(canvas, pose_status, cv::Point(popup.x + 24, popup.y + 186), 0.47, palette.text);

  putPanelText(canvas, "Radar points file", cv::Point(popup.x + 24, popup.y + 242), 0.45, palette.text_muted);
  putPanelText(canvas, "Map points file", cv::Point(popup.x + 340, popup.y + 242), 0.45, palette.text_muted);
  drawRadarButton(
    canvas,
    ui_state,
    radarDataFileButtonRect(ui_state),
    ui_state.radar_data_file.empty() ? "Select radar file" : shortenMiddle(ui_state.radar_data_file, 24));
  drawRadarButton(
    canvas,
    ui_state,
    radarPointsFileButtonRect(ui_state),
    ui_state.radar_points_file.empty() ? "Select map file" : shortenMiddle(ui_state.radar_points_file, 24));

  drawRadarDropdown(canvas, ui_state);

  putPanelText(canvas, "Registration pairs points by file order.", cv::Point(popup.x + 24, popup.y + popup.height - 46), 0.42, palette.text_muted);
  drawRadarButton(canvas, ui_state, radarRegisterButtonRect(ui_state), "Register");
  drawRadarButton(canvas, ui_state, radarCloseButtonRect(ui_state), "Close");
}

}  // namespace navigation::ui
