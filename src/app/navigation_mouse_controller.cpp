#include "app/navigation_mouse_controller.hpp"

#include <algorithm>
#include <cmath>

#include "keyboards/navigation_input_handler.hpp"
#include "opencv2/highgui.hpp"

namespace navigation::app
{
namespace
{

constexpr int kPointGroupDragThresholdPx = 6;
constexpr double kPointGroupWheelRotationRadFine = 3.14159265358979323846 / 180.0;   // ~1°  per click
constexpr double kPointGroupWheelRotationRadCoarse = 3.14159265358979323846 / 36.0;  // ~5°  per click

}  // namespace

NavigationMouseController::NavigationMouseController(
  NavigationNodeContext & context,
  NavigationUiCoordinator & ui_coordinator,
  NavigationPointsWorkflow & points_workflow,
  rclcpp::Logger logger)
: context_(context),
  ui_coordinator_(ui_coordinator),
  points_workflow_(points_workflow),
  logger_(logger)
{
}

void NavigationMouseController::setCtrlPressed(bool pressed)
{
  ctrl_pressed_ = pressed;
}

void NavigationMouseController::handleMouseEvent(int event, int x, int y, int flags)
{
  if (event != cv::EVENT_MOUSEWHEEL && event != cv::EVENT_MOUSEHWHEEL) {
    rememberMousePosition(x, y);
  }

  if (event == cv::EVENT_MOUSEWHEEL || event == cv::EVENT_MOUSEHWHEEL) {
    handleMouseWheel(x, y, flags);
  } else if (event == cv::EVENT_MBUTTONDOWN) {
    handleMiddleDown(x, y);
  } else if (event == cv::EVENT_MOUSEMOVE) {
    handleMouseMove(x, y, flags);
  } else if (event == cv::EVENT_MBUTTONUP) {
    handleMiddleUp();
  } else if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_LBUTTONDBLCLK) {
    handleLeftClick(x, y, flags);
  } else if (event == cv::EVENT_LBUTTONUP) {
    handleLeftUp(x, y);
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    handleRightClick(x, y);
  }
}

void NavigationMouseController::handleMouseWheel(int x, int y, int flags)
{
  int delta = cv::getMouseWheelDelta(flags);

  // Work around an OpenCV GTK backend quirk: the GTK backend encodes
  // scroll delta as +/-1 per click (with inverted sign), whereas the Qt
  // backend and the OpenCV documentation specify multiples of 120.
  if (std::abs(delta) < 10) {
    delta = -delta * 120;
  }

  const bool shift_held = (flags & cv::EVENT_FLAG_SHIFTKEY) != 0;
  handleWheelDelta(x, y, delta, shift_held);
}

void NavigationMouseController::handleWheelDelta(int x, int y, int delta, bool shift_held)
{
  if (delta == 0) {
    return;
  }

  if (context_.point_group_edit_active) {
    const double clicks = static_cast<double>(delta) / 120.0;
    const double angle = shift_held
      ? clicks * kPointGroupWheelRotationRadCoarse   // ~5° per click
      : clicks * kPointGroupWheelRotationRadFine;     // ~1° per click
    points_workflow_.rotateSelectedPointGroup(angle);
    rememberMousePosition(x, y);
    return;
  }

  if (context_.point_group_selection_drag_active) {
    return;
  }

  if (context_.input_mode != navigation::keyboards::TextInputMode::None ||
    context_.params_session.active() ||
    context_.segment_speed_edit_active ||
    context_.settings_popup_active ||
    context_.radar_popup_active ||
    context_.point_group_selection_drag_active ||
    context_.point_group_edit_active)
  {
    return;
  }

  if (!context_.panel_collapsed && x >= context_.map_width_px) {
    const int content_height = 900;
    const int max_scroll = std::max(0, content_height - context_.map_height_px);
    context_.panel_scroll_px =
      std::clamp(context_.panel_scroll_px - delta / 2, 0, max_scroll);
    context_.status_message = "Controls scrolled";
    rememberMousePosition(x, y);
    return;
  }

  const double factor = std::pow(1.15, static_cast<double>(delta) / 120.0);
  bool zoomed = context_.map->zoomAt(x, y, factor);
  if (!zoomed && has_last_mouse_position_) {
    zoomed = context_.map->zoomAt(last_mouse_x_, last_mouse_y_, factor);
  }

  if (zoomed) {
    rememberMousePosition(x, y);
    context_.status_message = delta > 0 ? "Map zoomed in" : "Map zoomed out";
  }
}

void NavigationMouseController::rememberMousePosition(int x, int y)
{
  if (x < 0 || y < 0) {
    return;
  }

  last_mouse_x_ = x;
  last_mouse_y_ = y;
  has_last_mouse_position_ = true;
}

void NavigationMouseController::handleMiddleDown(int x, int y)
{
  if (context_.input_mode != navigation::keyboards::TextInputMode::None ||
    context_.params_session.active() ||
    context_.segment_speed_edit_active ||
    context_.settings_popup_active ||
    context_.radar_popup_active ||
    context_.point_group_selection_drag_active ||
    context_.point_group_edit_active)
  {
    return;
  }

  const auto hit = context_.map->hitTestUi(x, y, ui_coordinator_.buildUiState());
  if (hit.action != navigation::ui::MapUiAction::None) {
    return;
  }

  if (context_.dropdown_mode != navigation::ui::MapDropdownMode::None) {
    ui_coordinator_.clearDropdown();
    context_.status_message = "Selection cancelled";
  }

  map_dragging_ = true;
  drag_last_x_ = x;
  drag_last_y_ = y;
}

void NavigationMouseController::handleMouseMove(int x, int y, int flags)
{
  (void)flags;  // EVENT_FLAG_MBUTTON is unreliable with the GTK backend.
  if (ctrl_left_dragging_) {
    points_workflow_.updatePointGroupSelectionDrag(x, y);
    return;
  }

  if (!map_dragging_) {
    return;
  }

  const int delta_x = x - drag_last_x_;
  const int delta_y = y - drag_last_y_;
  drag_last_x_ = x;
  drag_last_y_ = y;
  context_.map->panBy(delta_x, delta_y);
}

void NavigationMouseController::handleMiddleUp()
{
  map_dragging_ = false;
}

void NavigationMouseController::handleLeftClick(int x, int y, int flags)
{
  if (context_.input_mode != navigation::keyboards::TextInputMode::None) {
    const auto hit = context_.map->hitTestUi(x, y, ui_coordinator_.buildUiState());
    if (hit.action == navigation::ui::MapUiAction::InputClose) {
      ui_coordinator_.handleUiHit(hit);
    }
    return;
  }

  if (context_.point_group_edit_active) {
    context_.status_message = "WASD 10cm / Shift+WASD 2cm / wheel 1deg / Shift+wheel 5deg / Enter / Esc";
    return;
  }

  const auto hit = context_.map->hitTestUi(x, y, ui_coordinator_.buildUiState());
  if (context_.route_patch_active) {
    if (hit.action != navigation::ui::MapUiAction::None) {
      context_.status_message = "Finish route patch first";
      return;
    }
    points_workflow_.addClickedPoint(x, y);
    return;
  }

  if (context_.params_session.active()) {
    if (hit.action == navigation::ui::MapUiAction::ParamOption) {
      context_.params_session.selectByIndex(hit.option_index, context_.param_fields, context_.status_message);
    } else if (hit.action == navigation::ui::MapUiAction::ParamSave ||
      hit.action == navigation::ui::MapUiAction::ParamLoad ||
      hit.action == navigation::ui::MapUiAction::ParamClose)
    {
      ui_coordinator_.handleUiHit(hit);
    }
    return;
  }

  if (context_.segment_speed_edit_active) {
    if (hit.action == navigation::ui::MapUiAction::SegmentSpeedField ||
      hit.action == navigation::ui::MapUiAction::SegmentSpeedApply ||
      hit.action == navigation::ui::MapUiAction::SegmentSpeedClear ||
      hit.action == navigation::ui::MapUiAction::SegmentSpeedClose)
    {
      ui_coordinator_.handleUiHit(hit);
    }
    return;
  }

  if (hit.action != navigation::ui::MapUiAction::None) {
    if (hit.action == navigation::ui::MapUiAction::DropdownOption) {
      ui_coordinator_.handleDropdownClick(hit.option_index, (flags & cv::EVENT_FLAG_CTRLKEY) != 0);
    } else {
      ui_coordinator_.handleUiHit(hit);
    }
    return;
  }

  if (context_.dropdown_mode != navigation::ui::MapDropdownMode::None) {
    ui_coordinator_.clearDropdown();
    context_.status_message = "Selection cancelled";
    return;
  }

  // Enter point-group selection drag when the panel "Select Points" mode
  // is toggled on, or when Ctrl is held (keyboard shortcut fallback).
  if (context_.point_group_select_mode_active ||
      ctrl_pressed_ ||
      (flags & cv::EVENT_FLAG_CTRLKEY) != 0)
  {
    ctrl_left_start_x_ = x;
    ctrl_left_start_y_ = y;
    points_workflow_.beginPointGroupSelectionDrag(x, y);
    ctrl_left_dragging_ = context_.point_group_selection_drag_active;
    return;
  }

  points_workflow_.addClickedPoint(x, y);
}

void NavigationMouseController::handleLeftUp(int x, int y)
{
  if (!ctrl_left_dragging_) {
    // Clear sticky Ctrl state on any left button release so that a
    // stale ctrl_pressed_ (set via keyboard event) does not leak into
    // the next click.
    ctrl_pressed_ = false;
    return;
  }
  if (!context_.point_group_selection_drag_active) {
    ctrl_left_dragging_ = false;
    ctrl_pressed_ = false;
    return;
  }

  const int dx = x - ctrl_left_start_x_;
  const int dy = y - ctrl_left_start_y_;
  const bool dragged =
    dx * dx + dy * dy >= kPointGroupDragThresholdPx * kPointGroupDragThresholdPx;

  if (dragged) {
    points_workflow_.finishPointGroupSelectionDrag(x, y);
  } else {
    points_workflow_.cancelPointGroupSelectionDrag();
    // Only fall back to Ctrl+click shortcuts (event label / segment speed)
    // when the drag was initiated via the Ctrl key, not the panel toggle.
    if (!context_.point_group_select_mode_active) {
      handleCtrlLeftClick(x, y);
    }
  }

  ctrl_left_dragging_ = false;
  ctrl_pressed_ = false;
}

void NavigationMouseController::handleCtrlLeftClick(int x, int y)
{
  const int point_index = context_.map->hitTestPoint(x, y, 14);
  if (point_index >= 0) {
    ui_coordinator_.beginEventLabelInput(static_cast<std::size_t>(point_index));
    return;
  }
  const int segment_target_index = context_.map->hitTestSegmentTarget(x, y, 10);
  if (segment_target_index >= 0) {
    ui_coordinator_.beginSegmentSpeedInput(static_cast<std::size_t>(segment_target_index));
    return;
  }
  context_.status_message = "Ctrl+drag to select points, or Ctrl+click point/segment";
}

void NavigationMouseController::handleRightClick(int x, int y)
{
  if (context_.input_mode != navigation::keyboards::TextInputMode::None ||
    context_.params_session.active() ||
    context_.segment_speed_edit_active ||
    context_.settings_popup_active ||
    context_.radar_popup_active)
  {
    return;
  }

  if (context_.dropdown_mode != navigation::ui::MapDropdownMode::None) {
    ui_coordinator_.clearDropdown();
    context_.status_message = "Selection cancelled";
    return;
  }

  points_workflow_.removeNearestPoint(x, y);
}

}  // namespace navigation::app
