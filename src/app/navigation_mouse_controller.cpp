#include "app/navigation_mouse_controller.hpp"

#include <algorithm>
#include <cmath>

#include "keyboards/navigation_input_handler.hpp"
#include "opencv2/highgui.hpp"

namespace navigation::app
{

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

  handleWheelDelta(x, y, delta);
}

void NavigationMouseController::handleWheelDelta(int x, int y, int delta)
{
  if (context_.input_mode != navigation::keyboards::TextInputMode::None ||
    context_.params_session.active() ||
    context_.segment_speed_edit_active ||
    context_.settings_popup_active ||
    context_.radar_popup_active)
  {
    return;
  }

  if (delta == 0) {
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
    context_.radar_popup_active)
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

  if ((flags & cv::EVENT_FLAG_CTRLKEY) != 0) {
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
    context_.status_message = "Ctrl+click a point for event or a segment for speed";
    return;
  }

  points_workflow_.addClickedPoint(x, y);
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
