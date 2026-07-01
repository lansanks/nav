#include "app/navigation_ui_coordinator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "calibration/radar_calibration.hpp"
#include "keyboards/navigation_input_handler.hpp"
#include "keyboards/navigation_keys.hpp"
#include "maps/navigation_map_helpers.hpp"
#include "maps/point_store.hpp"
#include "optim/task_order_planner.hpp"
#include "params/navigation_params.hpp"

namespace navigation::app
{
namespace
{

std::string trimCopy(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string defaultMergedPointsName()
{
  const std::string base = "merged_points";
  for (int index = 1; index < 1000; ++index) {
    const auto name = index == 1 ? base + ".yaml" : base + "_" + std::to_string(index) + ".yaml";
    std::error_code error;
    if (!std::filesystem::exists(navigation::maps::resolvePointsFilePath(name), error)) {
      return name;
    }
  }
  return base + ".yaml";
}

bool parseDoubleField(const std::string & text, const std::string & label, double & value, std::string & error)
{
  const auto trimmed = trimCopy(text);
  if (trimmed.empty()) {
    error = label + " is empty";
    return false;
  }
  try {
    std::size_t consumed = 0;
    value = std::stod(trimmed, &consumed);
    if (consumed != trimmed.size()) {
      error = label + " has trailing text";
      return false;
    }
  } catch (const std::exception &) {
    error = label + " is invalid";
    return false;
  }
  if (value < 0.0) {
    error = label + " must be >= 0";
    return false;
  }
  return true;
}

bool parseIntField(const std::string & text, const std::string & label, int & value, std::string & error)
{
  const auto trimmed = trimCopy(text);
  if (trimmed.empty()) {
    error = label + " is empty";
    return false;
  }
  try {
    std::size_t consumed = 0;
    value = std::stoi(trimmed, &consumed);
    if (consumed != trimmed.size()) {
      error = label + " has trailing text";
      return false;
    }
  } catch (const std::exception &) {
    error = label + " is invalid";
    return false;
  }
  return true;
}

bool parseSignedDoubleField(const std::string & text, const std::string & label, double & value, std::string & error)
{
  const auto trimmed = trimCopy(text);
  if (trimmed.empty()) {
    error = label + " is empty";
    return false;
  }
  try {
    std::size_t consumed = 0;
    value = std::stod(trimmed, &consumed);
    if (consumed != trimmed.size()) {
      error = label + " has trailing text";
      return false;
    }
  } catch (const std::exception &) {
    error = label + " is invalid";
    return false;
  }
  return true;
}

double segmentSpeedFactorForLevel(std::uint8_t level)
{
  constexpr std::array<double, 7> factors{0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00};
  if (level < 1 || level > factors.size()) {
    return factors[2];
  }
  return factors[static_cast<std::size_t>(level - 1)];
}

bool parseSlotCategories(const std::string & text, std::vector<int> & categories, std::string & error)
{
  categories.clear();
  for (const char ch : text) {
    if (ch >= '0' && ch <= '3') {
      categories.push_back(ch - '0');
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',' || ch == ';' || ch == '|') {
      continue;
    }
    error = "Slot categories must use only 0..3";
    return false;
  }

  if (categories.size() != 8) {
    error = "Enter 8 slot categories for slots 0..7";
    return false;
  }
  return true;
}

bool parseHighScoreCategory(const std::string & text, std::optional<int> & high_score, std::string & error)
{
  const auto trimmed = trimCopy(text);
  if (trimmed.empty() || trimmed == "-1" || trimmed == "none" || trimmed == "None" || trimmed == "NONE") {
    high_score = std::nullopt;
    return true;
  }

  try {
    std::size_t consumed = 0;
    const int value = std::stoi(trimmed, &consumed);
    if (consumed != trimmed.size() || value < 0 || value > 3) {
      error = "High score category must be -1 or 0..3";
      return false;
    }
    high_score = value;
    return true;
  } catch (const std::exception &) {
    error = "High score category is invalid";
    return false;
  }
}

std::string formatPlanOrder(const std::vector<int> & order)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < order.size(); ++i) {
    if (i > 0) {
      stream << "->";
    }
    stream << order[i];
  }
  return stream.str();
}

navigation::ui::MapPlanDisplayMode nextPlanDisplayMode(
  navigation::ui::MapPlanDisplayMode mode)
{
  using navigation::ui::MapPlanDisplayMode;
  if (mode == MapPlanDisplayMode::Full) {
    return MapPlanDisplayMode::OrderOnly;
  }
  if (mode == MapPlanDisplayMode::OrderOnly) {
    return MapPlanDisplayMode::Hidden;
  }
  return MapPlanDisplayMode::Full;
}

std::string planDisplayStatus(navigation::ui::MapPlanDisplayMode mode)
{
  using navigation::ui::MapPlanDisplayMode;
  if (mode == MapPlanDisplayMode::Full) {
    return "Mission plan visible";
  }
  if (mode == MapPlanDisplayMode::OrderOnly) {
    return "Mission plan order only";
  }
  return "Mission plan hidden";
}

using OptimPoint = navigation::optim::Point2;

constexpr double kMissionLaneOffset = 0.05;
constexpr double kMissionRowTolerance = 0.12;
constexpr double kMissionEps = 1.0e-9;
constexpr double kPointGroupMoveStepM = 0.02;
constexpr double kPointGroupMoveStepCoarseM = 0.10;

enum class LaneSide { Left, Right };

struct CandidatePoint
{
  OptimPoint point;
  std::uint8_t task_type{navigation::maps::kTaskTypeNone};
};

double distance(OptimPoint a, OptimPoint b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

OptimPoint add(OptimPoint a, OptimPoint b)
{
  return {a.x + b.x, a.y + b.y};
}

OptimPoint subtract(OptimPoint a, OptimPoint b)
{
  return {a.x - b.x, a.y - b.y};
}

OptimPoint multiply(OptimPoint point, double scale)
{
  return {point.x * scale, point.y * scale};
}

double dot(OptimPoint a, OptimPoint b)
{
  return a.x * b.x + a.y * b.y;
}

double cross(OptimPoint a, OptimPoint b)
{
  return a.x * b.y - a.y * b.x;
}

OptimPoint normalizeOr(OptimPoint point, OptimPoint fallback)
{
  const double norm = std::hypot(point.x, point.y);
  if (norm <= kMissionEps) {
    return fallback;
  }
  return {point.x / norm, point.y / norm};
}

LaneSide opposite(LaneSide side)
{
  return side == LaneSide::Left ? LaneSide::Right : LaneSide::Left;
}

double laneSign(LaneSide side)
{
  return side == LaneSide::Left ? -1.0 : 1.0;
}

OptimPoint centroid(const std::vector<navigation::optim::PlanningMapPoint> & points)
{
  OptimPoint center;
  if (points.empty()) {
    return center;
  }
  for (const auto & point : points) {
    center.x += point.position.x;
    center.y += point.position.y;
  }
  const double count = static_cast<double>(points.size());
  center.x /= count;
  center.y /= count;
  return center;
}

OptimPoint lanePoint(
  OptimPoint center,
  OptimPoint side_direction,
  LaneSide lane,
  double side_distance)
{
  return {
    center.x + side_direction.x * side_distance + laneSign(lane) * kMissionLaneOffset,
    center.y + side_direction.y * side_distance,
  };
}

void appendMapPoint(
  std::vector<navigation::maps::MapPoint> & route,
  int & next_id,
  OptimPoint point,
  std::uint8_t task_type)
{
  navigation::maps::MapPoint map_point;
  map_point.id = next_id++;
  map_point.x = point.x;
  map_point.y = point.y;
  map_point.fast = task_type != navigation::maps::kTaskTypeNone;
  map_point.constant_speed = false;
  map_point.task_type = task_type;
  route.push_back(map_point);
}

double candidateCost(
  OptimPoint previous,
  const std::vector<CandidatePoint> & candidate,
  const std::optional<OptimPoint> & next)
{
  if (candidate.empty()) {
    return next ? distance(previous, *next) : 0.0;
  }

  double cost = distance(previous, candidate.front().point);
  for (std::size_t i = 1; i < candidate.size(); ++i) {
    cost += distance(candidate[i - 1].point, candidate[i].point);
  }
  if (next) {
    cost += distance(candidate.back().point, *next);
  }
  return cost;
}

bool segmentsProperlyIntersect(OptimPoint a, OptimPoint b, OptimPoint c, OptimPoint d)
{
  const double ab_c = cross(subtract(c, a), subtract(b, a));
  const double ab_d = cross(subtract(d, a), subtract(b, a));
  const double cd_a = cross(subtract(a, c), subtract(d, c));
  const double cd_b = cross(subtract(b, c), subtract(d, c));
  return ab_c * ab_d < -kMissionEps && cd_a * cd_b < -kMissionEps;
}

int candidateCrossingCount(
  OptimPoint previous,
  const std::vector<CandidatePoint> & candidate,
  const std::optional<OptimPoint> & next)
{
  std::vector<OptimPoint> vertices;
  vertices.reserve(candidate.size() + 2);
  vertices.push_back(previous);
  for (const auto & point : candidate) {
    vertices.push_back(point.point);
  }
  if (next) {
    vertices.push_back(*next);
  }

  int crossings = 0;
  if (vertices.size() < 4) {
    return crossings;
  }
  for (std::size_t i = 0; i + 1 < vertices.size(); ++i) {
    for (std::size_t j = i + 2; j + 1 < vertices.size(); ++j) {
      if (segmentsProperlyIntersect(
          vertices[i],
          vertices[i + 1],
          vertices[j],
          vertices[j + 1]))
      {
        ++crossings;
      }
    }
  }
  return crossings;
}

double candidateEntryCost(
  OptimPoint previous,
  const std::vector<CandidatePoint> & candidate)
{
  if (candidate.size() < 2) {
    return candidateCost(previous, candidate, std::nullopt);
  }
  return distance(previous, candidate[0].point) +
    distance(candidate[0].point, candidate[1].point);
}

std::vector<CandidatePoint> serviceCandidate(
  OptimPoint center,
  OptimPoint side_direction,
  LaneSide red_lane,
  bool include_exit_lane,
  std::uint8_t task_type,
  double near_distance,
  double far_distance)
{
  const LaneSide exit_lane = opposite(red_lane);
  std::vector<CandidatePoint> points;
  points.push_back({
      lanePoint(center, side_direction, red_lane, far_distance),
      navigation::maps::kTaskTypeNone});
  points.push_back({lanePoint(center, side_direction, red_lane, near_distance), task_type});
  if (include_exit_lane) {
    points.push_back({
        lanePoint(center, side_direction, exit_lane, near_distance),
        navigation::maps::kTaskTypeNone});
  }
  return points;
}

std::vector<CandidatePoint> chooseServiceCandidate(
  OptimPoint center,
  OptimPoint side_direction,
  OptimPoint previous,
  const std::optional<OptimPoint> & next,
  std::uint8_t task_type,
  double near_distance,
  double far_distance)
{
  const bool include_exit_lane = next.has_value();
  std::vector<CandidatePoint> best;
  int best_crossings = std::numeric_limits<int>::max();
  double best_entry_cost = std::numeric_limits<double>::infinity();
  double best_total_cost = std::numeric_limits<double>::infinity();
  for (const LaneSide lane : {LaneSide::Left, LaneSide::Right}) {
    auto candidate = serviceCandidate(
      center,
      side_direction,
      lane,
      include_exit_lane,
      task_type,
      near_distance,
      far_distance);
    const int crossings = candidateCrossingCount(previous, candidate, next);
    const double entry_cost = candidateEntryCost(previous, candidate);
    const double total_cost = candidateCost(previous, candidate, next);
    if (crossings < best_crossings ||
      (crossings == best_crossings && entry_cost + kMissionEps < best_entry_cost) ||
      (crossings == best_crossings && std::abs(entry_cost - best_entry_cost) <= kMissionEps &&
      total_cost < best_total_cost))
    {
      best_crossings = crossings;
      best_entry_cost = entry_cost;
      best_total_cost = total_cost;
      best = std::move(candidate);
    }
  }
  return best;
}

void appendCandidate(
  std::vector<navigation::maps::MapPoint> & route,
  int & next_id,
  const std::vector<CandidatePoint> & candidate)
{
  for (const auto & point : candidate) {
    appendMapPoint(route, next_id, point.point, point.task_type);
  }
}

std::vector<navigation::optim::PlanningMapPoint> storageSlotsClosestToReturn(
  const navigation::optim::PlanningResult & result,
  OptimPoint return_center,
  double & near_row_y)
{
  std::vector<navigation::optim::PlanningMapPoint> slots;
  if (result.storage_slots.empty()) {
    near_row_y = 0.0;
    return slots;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  near_row_y = result.storage_slots.front().position.y;
  for (const auto & slot : result.storage_slots) {
    const double y_distance = std::abs(slot.position.y - return_center.y);
    if (y_distance < best_distance) {
      best_distance = y_distance;
      near_row_y = slot.position.y;
    }
  }

  for (const auto & slot : result.storage_slots) {
    if (std::abs(slot.position.y - near_row_y) <= kMissionRowTolerance) {
      slots.push_back(slot);
    }
  }
  std::sort(slots.begin(), slots.end(), [](const auto & left, const auto & right) {
    if (std::abs(left.position.x - right.position.x) > kMissionEps) {
      return left.position.x < right.position.x;
    }
    return left.id < right.id;
  });
  return slots;
}

std::optional<std::array<OptimPoint, 2>> firstFarRowCrossingBuffers(
  const navigation::optim::PlanningResult & result,
  const navigation::optim::PlanningStep & first_step,
  OptimPoint return_center,
  double near_row_y)
{
  const auto near_row_slots = storageSlotsClosestToReturn(result, return_center, near_row_y);
  if (near_row_slots.size() < 2) {
    return std::nullopt;
  }

  double target_x = first_step.box_position.x;
  const double dy = first_step.return_zone_position.y - first_step.box_position.y;
  if (std::abs(dy) > kMissionEps) {
    const double t = (near_row_y - first_step.box_position.y) / dy;
    target_x = first_step.box_position.x +
      (first_step.return_zone_position.x - first_step.box_position.x) * t;
  }

  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < near_row_slots.size(); ++i) {
    const double midpoint_x =
      0.5 * (near_row_slots[i].position.x + near_row_slots[i + 1].position.x);
    const double x_distance = std::abs(midpoint_x - target_x);
    if (x_distance < best_distance) {
      best_distance = x_distance;
      best_index = i;
    }
  }

  const auto left = near_row_slots[best_index].position;
  const auto right = near_row_slots[best_index + 1].position;
  const OptimPoint midpoint{0.5 * (left.x + right.x), 0.5 * (left.y + right.y)};
  const auto tangent = normalizeOr(subtract(right, left), {1.0, 0.0});
  auto normal = OptimPoint{-tangent.y, tangent.x};
  if (dot(normal, subtract(return_center, midpoint)) < 0.0) {
    normal = multiply(normal, -1.0);
  }

  const double row_gap = std::abs(first_step.box_position.y - near_row_y);
  const double offset = std::clamp(row_gap * 0.5, 0.32, 0.48);
  return std::array<OptimPoint, 2>{
    add(midpoint, multiply(normal, -offset)),
    add(midpoint, multiply(normal, offset)),
  };
}

std::vector<CandidatePoint> firstStartSidePickupCandidate(
  const navigation::optim::PlanningStep & step,
  OptimPoint start_position,
  LaneSide red_lane,
  double near_distance,
  double far_distance)
{
  const double start_side_y = start_position.y < step.box_position.y ? -1.0 : 1.0;
  const OptimPoint side_direction{0.0, start_side_y};
  return {
    {lanePoint(step.box_position, side_direction, red_lane, far_distance), navigation::maps::kTaskTypeNone},
    {lanePoint(step.box_position, side_direction, red_lane, near_distance), navigation::maps::kTaskTypePickup},
  };
}

std::vector<CandidatePoint> chooseFirstStartSidePickup(
  const navigation::optim::PlanningStep & step,
  OptimPoint start_position,
  const std::optional<OptimPoint> & next,
  double near_distance,
  double far_distance)
{
  std::vector<CandidatePoint> best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const LaneSide lane : {LaneSide::Left, LaneSide::Right}) {
    auto candidate = firstStartSidePickupCandidate(
      step,
      start_position,
      lane,
      near_distance,
      far_distance);
    const double cost = candidateCost(start_position, candidate, next);
    if (cost < best_cost) {
      best_cost = cost;
      best = std::move(candidate);
    }
  }
  return best;
}

std::vector<navigation::maps::MapPoint> buildMissionNavigationRoute(
  const navigation::optim::PlanningResult & result,
  double storage_near_distance,
  double storage_far_distance,
  double return_near_distance,
  double return_far_distance)
{
  std::vector<navigation::maps::MapPoint> route;
  if (result.steps.empty()) {
    return route;
  }

  const auto storage_center = centroid(result.storage_slots);
  const auto return_center = centroid(result.return_zones);
  const auto storage_to_return = normalizeOr(subtract(return_center, storage_center), {0.0, 1.0});
  const auto return_to_storage = multiply(storage_to_return, -1.0);

  double near_row_y = 0.0;
  storageSlotsClosestToReturn(result, return_center, near_row_y);

  int next_id = 1;
  OptimPoint cursor = result.start_position;

  for (std::size_t i = 0; i < result.steps.size(); ++i) {
    const auto & step = result.steps[i];
    const bool first_step = i == 0;
    const bool first_far_from_return =
      first_step && std::abs(step.box_position.y - near_row_y) > kMissionRowTolerance;

    if (first_far_from_return) {
      const auto crossing = firstFarRowCrossingBuffers(result, step, return_center, near_row_y);
      std::optional<OptimPoint> next_after_pickup;
      if (crossing) {
        next_after_pickup = (*crossing)[0];
      } else {
        next_after_pickup = step.return_zone_position;
      }
      appendCandidate(
        route,
        next_id,
        chooseFirstStartSidePickup(
          step,
          result.start_position,
          next_after_pickup,
          storage_near_distance,
          storage_far_distance));
      cursor = {route.back().x, route.back().y};
      if (crossing) {
        appendMapPoint(route, next_id, (*crossing)[0], navigation::maps::kTaskTypeNone);
        appendMapPoint(route, next_id, (*crossing)[1], navigation::maps::kTaskTypeNone);
        cursor = (*crossing)[1];
      }
    } else {
      appendCandidate(
        route,
        next_id,
        chooseServiceCandidate(
          step.box_position,
          storage_to_return,
          cursor,
          step.return_zone_position,
          navigation::maps::kTaskTypePickup,
          storage_near_distance,
          storage_far_distance));
      cursor = {route.back().x, route.back().y};
    }

    std::optional<OptimPoint> next_box;
    if (i + 1 < result.steps.size()) {
      next_box = result.steps[i + 1].box_position;
    }
    appendCandidate(
      route,
      next_id,
      chooseServiceCandidate(
        step.return_zone_position,
        return_to_storage,
        cursor,
        next_box,
        navigation::maps::kTaskTypePlace,
        return_near_distance,
        return_far_distance));
    cursor = {route.back().x, route.back().y};
  }

  return route;
}

}  // namespace

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
    handleDropdownClick(hit.option_index, false);
    return;
  }
  if (hit.action == navigation::ui::MapUiAction::SettingsField) {
    selectSettingsField(hit.option_index);
    return;
  }
  if (hit.action == navigation::ui::MapUiAction::SegmentSpeedField) {
    selectSegmentSpeedField(hit.option_index);
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
      context_.segment_speed_edit_active = false;
      context_.segment_speed_field_editing = false;
      context_.params_session.open(context_.status_message);
      break;
    case navigation::ui::MapUiAction::Settings:
      openSettings();
      break;
    case navigation::ui::MapUiAction::SettingsApply:
      applyMissionSettings();
      break;
    case navigation::ui::MapUiAction::SettingsClose:
      closeSettings();
      break;
    case navigation::ui::MapUiAction::SegmentSpeedApply:
      applySegmentSpeed();
      break;
    case navigation::ui::MapUiAction::SegmentSpeedClear:
      clearSegmentSpeed();
      break;
    case navigation::ui::MapUiAction::SegmentSpeedClose:
      closeSegmentSpeedEditor();
      break;
    case navigation::ui::MapUiAction::SegmentSpeedField:
      break;
    case navigation::ui::MapUiAction::Radar:
      clearDropdown();
      context_.params_session.setActive(false);
      context_.segment_speed_edit_active = false;
      context_.segment_speed_field_editing = false;
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
    case navigation::ui::MapUiAction::ToggleMissionPlan:
      context_.mission_plan_display_mode = nextPlanDisplayMode(context_.mission_plan_display_mode);
      context_.status_message = planDisplayStatus(context_.mission_plan_display_mode);
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
    case navigation::ui::MapUiAction::TogglePointGroupSelect:
      clearDropdown();
      context_.point_group_select_mode_active = !context_.point_group_select_mode_active;
      context_.status_message = context_.point_group_select_mode_active ?
        "Select mode ON: left-drag to select points" :
        "Select mode OFF";
      break;
    case navigation::ui::MapUiAction::UiOnly:
      if (context_.dropdown_mode != navigation::ui::MapDropdownMode::None) {
        clearDropdown();
        context_.status_message = "Selection cancelled";
      }
      break;
    case navigation::ui::MapUiAction::DropdownOption:
    case navigation::ui::MapUiAction::ParamOption:
    case navigation::ui::MapUiAction::SettingsField:
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
  context_.dropdown_marked_indices.clear();
  context_.dropdown_marked_order.clear();

  if (mode == navigation::ui::MapDropdownMode::LoadPoints) {
    context_.dropdown_paths = navigation::maps::listPointsFilesForGroup(
      navigation::maps::obstacleMapPointsGroup(context_.current_map_file),
      context_.points_file);
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
  context_.dropdown_marked_indices.clear();
  context_.dropdown_marked_order.clear();
  context_.dropdown_selected_index = -1;
}

void NavigationUiCoordinator::handleDropdownClick(int option_index, bool ctrl_pressed)
{
  if (context_.dropdown_mode == navigation::ui::MapDropdownMode::LoadPoints && ctrl_pressed) {
    togglePointFileMergeSelection(option_index);
    return;
  }
  selectDropdownOption(option_index);
}

void NavigationUiCoordinator::togglePointFileMergeSelection(int option_index)
{
  if (option_index < 0 || option_index >= static_cast<int>(context_.dropdown_paths.size())) {
    return;
  }

  context_.dropdown_selected_index = option_index;
  const auto marked_iter = std::find(
    context_.dropdown_marked_indices.begin(),
    context_.dropdown_marked_indices.end(),
    option_index);
  if (marked_iter == context_.dropdown_marked_indices.end()) {
    context_.dropdown_marked_indices.push_back(option_index);
    context_.dropdown_marked_order.push_back(option_index);
  } else {
    context_.dropdown_marked_indices.erase(marked_iter);
    context_.dropdown_marked_order.erase(
      std::remove(
        context_.dropdown_marked_order.begin(),
        context_.dropdown_marked_order.end(),
        option_index),
      context_.dropdown_marked_order.end());
  }

  context_.status_message =
    "Selected " + std::to_string(context_.dropdown_marked_order.size()) + " point files";
}

void NavigationUiCoordinator::beginMergeSelectedPointFiles()
{
  if (context_.dropdown_mode != navigation::ui::MapDropdownMode::LoadPoints ||
    context_.dropdown_marked_order.size() < 2)
  {
    context_.status_message = "Select at least two point files";
    return;
  }

  context_.pending_merge_point_paths.clear();
  for (const auto option_index : context_.dropdown_marked_order) {
    if (option_index < 0 || option_index >= static_cast<int>(context_.dropdown_paths.size())) {
      context_.status_message = "Merge selection invalid";
      context_.pending_merge_point_paths.clear();
      return;
    }
    context_.pending_merge_point_paths.push_back(context_.dropdown_paths[static_cast<std::size_t>(option_index)]);
  }

  beginTextInput(
    navigation::keyboards::TextInputMode::MergePointsAs,
    "Merged point file name",
    defaultMergedPointsName());
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
  context_.segment_speed_edit_active = false;
  context_.segment_speed_field_editing = false;
  context_.input_mode = mode;
  context_.input_label = label;
  context_.input_text = default_text;
  context_.status_message = "Typing input";
}

void NavigationUiCoordinator::beginEventLabelInput(std::size_t point_index)
{
  if (point_index >= context_.map->points().size()) {
    return;
  }

  context_.event_label_edit_index = point_index;
  context_.event_label_edit_active = true;
  const auto & point = context_.map->points()[point_index];
  beginTextInput(
    navigation::keyboards::TextInputMode::EventLabel,
    "Event label (@stop_1, @back_20, @end_3) for point " + std::to_string(point.id),
    point.event_label);
}

void NavigationUiCoordinator::beginSegmentSpeedInput(std::size_t target_index)
{
  if (target_index == 0 || target_index >= context_.map->points().size()) {
    return;
  }

  clearDropdown();
  resetTextInput();
  context_.params_session.setActive(false);
  context_.settings_popup_active = false;
  context_.radar_popup_active = false;
  context_.segment_speed_edit_target_index = target_index;
  context_.segment_speed_edit_active = true;
  context_.segment_speed_selected_index = 0;
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
  const auto & from = context_.map->points()[target_index - 1];
  const auto & to = context_.map->points()[target_index];
  context_.segment_speed_title =
    "Segment " + std::to_string(from.id) + "-" + std::to_string(to.id) + " Speed";
  if (to.segment_custom_speed) {
    context_.segment_speed_level =
      to.segment_speed_level >= 1 && to.segment_speed_level <= 7 ? to.segment_speed_level : 3;
    context_.segment_speed_constant_mode = to.segment_constant_speed;
    context_.segment_speed_linear_x = to.segment_linear_x;
    context_.segment_speed_max_angular_speed = to.segment_max_angular_speed;
    context_.segment_speed_k_alpha = to.segment_k_alpha;
    context_.segment_speed_k_beta = to.segment_k_beta;
  } else {
    context_.segment_speed_level = 3;
    context_.segment_speed_constant_mode = true;
    applySegmentSpeedLevelDefaults();
  }
  context_.status_message = "Segment speed editor";
}

void NavigationUiCoordinator::cancelTextInput()
{
  context_.event_label_edit_active = false;
  context_.segment_speed_edit_active = false;
  context_.pending_merge_point_paths.clear();
  resetTextInput();
  context_.status_message = "Input cancelled";
}

void NavigationUiCoordinator::confirmTextInput()
{
  const auto value = context_.input_text;
  const auto mode = context_.input_mode;
  resetTextInput();

  if (mode == navigation::keyboards::TextInputMode::EventLabel) {
    if (context_.event_label_edit_active) {
      points_workflow_.setEventLabel(context_.event_label_edit_index, value);
    }
    context_.event_label_edit_active = false;
    return;
  }

  if (mode == navigation::keyboards::TextInputMode::SegmentSpeed) {
    if (context_.segment_speed_edit_active) {
      points_workflow_.setSegmentSpeed(context_.segment_speed_edit_target_index, value);
    }
    context_.segment_speed_edit_active = false;
    return;
  }

  if (value.empty()) {
    context_.status_message = "Input is empty";
    return;
  }

  if (mode == navigation::keyboards::TextInputMode::NewPoints) {
    points_workflow_.createNewPointsFile(value);
  } else if (mode == navigation::keyboards::TextInputMode::SavePointsAs) {
    points_workflow_.savePointsAs(value);
  } else if (mode == navigation::keyboards::TextInputMode::MergePointsAs) {
    points_workflow_.mergePointsFilesAs(context_.pending_merge_point_paths, value);
    context_.pending_merge_point_paths.clear();
  } else if (mode == navigation::keyboards::TextInputMode::SaveParamsAs) {
    saveParamsAs(value);
  } else if (mode == navigation::keyboards::TextInputMode::SaveRadarPointAs) {
    saveRadarPointAs(value);
  }
  context_.event_label_edit_active = false;
  context_.segment_speed_edit_active = false;
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

bool NavigationUiCoordinator::handleSegmentSpeedKey(int key)
{
  if (!context_.segment_speed_edit_active) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (context_.segment_speed_field_editing) {
    if (navigation::keyboards::isEscKey(key)) {
      context_.segment_speed_field_editing = false;
      context_.segment_speed_edit_text.clear();
      context_.status_message = "Segment field edit cancelled";
      return true;
    }
    if (navigation::keyboards::isEnterKey(key)) {
      applySegmentSpeedFieldEdit();
      return true;
    }
    if (navigation::keyboards::isBackspaceKey(key)) {
      if (!context_.segment_speed_edit_text.empty()) {
        context_.segment_speed_edit_text.pop_back();
      }
      return true;
    }
    if (navigation::keyboards::isArrowKey(key)) {
      return true;
    }

    const int ascii = navigation::keyboards::keyAscii(key);
    if ((ascii >= '0' && ascii <= '9') || ascii == '.' || ascii == '-' || ascii == '+') {
      context_.segment_speed_edit_text.push_back(static_cast<char>(ascii));
    }
    return true;
  }

  if (navigation::keyboards::isEscKey(key)) {
    closeSegmentSpeedEditor();
    return true;
  }
  if (navigation::keyboards::isEnterKey(key)) {
    if (context_.segment_speed_selected_index == 0) {
      context_.segment_speed_constant_mode = !context_.segment_speed_constant_mode;
      context_.status_message = context_.segment_speed_constant_mode ?
        "Segment mode: constant speed" :
        "Segment mode: P control";
    } else if (context_.segment_speed_selected_index == 1) {
      applySegmentSpeedLevelDefaults();
    } else {
      beginSegmentSpeedFieldEdit();
    }
    return true;
  }
  if (navigation::keyboards::isUpKey(key)) {
    context_.segment_speed_selected_index = std::max(0, context_.segment_speed_selected_index - 1);
    return true;
  }
  if (navigation::keyboards::isDownKey(key)) {
    const int max_index = static_cast<int>(segmentSpeedFieldNames().size()) - 1;
    context_.segment_speed_selected_index = std::min(max_index, context_.segment_speed_selected_index + 1);
    return true;
  }

  const int ascii = navigation::keyboards::keyAscii(key);
  if (context_.segment_speed_selected_index == 1 && ascii >= '1' && ascii <= '7') {
    context_.segment_speed_level = static_cast<std::uint8_t>(ascii - '0');
    applySegmentSpeedLevelDefaults();
    return true;
  }
  if (ascii == 'a' || ascii == 'A') {
    applySegmentSpeed();
    return true;
  }
  if (ascii == 'c' || ascii == 'C') {
    clearSegmentSpeed();
    return true;
  }
  return true;
}

bool NavigationUiCoordinator::handlePointGroupEditKey(int key)
{
  if (context_.point_group_selection_drag_active) {
    if (key == -1) {
      return true;
    }
    if (navigation::keyboards::isEscKey(key)) {
      points_workflow_.cancelPointGroupSelectionDrag();
      return true;
    }
    return true;
  }

  if (!context_.point_group_edit_active) {
    // When selection mode is toggled on but no drag/edit is active,
    // Esc exits selection mode so the user can return to normal clicks.
    if (context_.point_group_select_mode_active && navigation::keyboards::isEscKey(key)) {
      context_.point_group_select_mode_active = false;
      context_.status_message = "Select mode OFF";
      return true;
    }
    return false;
  }

  if (key == -1) {
    return true;
  }

  const int ascii = navigation::keyboards::keyAscii(key);

  if (navigation::keyboards::isEscKey(key)) {
    points_workflow_.cancelPointGroupEdit();
    return true;
  }
  if (navigation::keyboards::isEnterKey(key)) {
    points_workflow_.confirmPointGroupEdit();
    return true;
  }
  // WASD coarse (10 cm) — lowercase without Shift
  if (ascii == 'a') {
    points_workflow_.moveSelectedPointGroup(-kPointGroupMoveStepCoarseM, 0.0);
    return true;
  }
  if (ascii == 'd') {
    points_workflow_.moveSelectedPointGroup(kPointGroupMoveStepCoarseM, 0.0);
    return true;
  }
  if (ascii == 'w') {
    points_workflow_.moveSelectedPointGroup(0.0, kPointGroupMoveStepCoarseM);
    return true;
  }
  if (ascii == 's') {
    points_workflow_.moveSelectedPointGroup(0.0, -kPointGroupMoveStepCoarseM);
    return true;
  }
  // Shift+WASD fine (2 cm) — uppercase
  if (ascii == 'A') {
    points_workflow_.moveSelectedPointGroup(-kPointGroupMoveStepM, 0.0);
    return true;
  }
  if (ascii == 'D') {
    points_workflow_.moveSelectedPointGroup(kPointGroupMoveStepM, 0.0);
    return true;
  }
  if (ascii == 'W') {
    points_workflow_.moveSelectedPointGroup(0.0, kPointGroupMoveStepM);
    return true;
  }
  if (ascii == 'S') {
    points_workflow_.moveSelectedPointGroup(0.0, -kPointGroupMoveStepM);
    return true;
  }

  context_.status_message = "WASD 10cm / Shift+WASD 2cm / wheel 1deg / Shift+wheel 5deg / Enter / Esc";
  return true;
}

bool NavigationUiCoordinator::handleSettingsKey(int key)
{
  if (!context_.settings_popup_active) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (context_.settings_editing) {
    if (navigation::keyboards::isEscKey(key)) {
      context_.settings_editing = false;
      context_.settings_edit_text.clear();
      context_.status_message = "Setting edit cancelled";
      return true;
    }
    if (navigation::keyboards::isEnterKey(key)) {
      applySettingsEdit();
      return true;
    }
    if (navigation::keyboards::isBackspaceKey(key)) {
      if (!context_.settings_edit_text.empty()) {
        context_.settings_edit_text.pop_back();
      }
      return true;
    }
    if (navigation::keyboards::isArrowKey(key)) {
      return true;
    }

    const int ascii = navigation::keyboards::keyAscii(key);
    if (ascii >= 32 && ascii <= 126) {
      context_.settings_edit_text.push_back(static_cast<char>(ascii));
    }
    return true;
  }

  if (navigation::keyboards::isEscKey(key)) {
    closeSettings();
    return true;
  }
  if (navigation::keyboards::isEnterKey(key)) {
    beginSettingsEdit();
    return true;
  }
  if (navigation::keyboards::isUpKey(key)) {
    context_.settings_selected_index = std::max(0, context_.settings_selected_index - 1);
    return true;
  }
  if (navigation::keyboards::isDownKey(key)) {
    const int max_index = static_cast<int>(settingsFieldNames().size()) - 1;
    context_.settings_selected_index = std::min(max_index, context_.settings_selected_index + 1);
    return true;
  }

  const int ascii = navigation::keyboards::keyAscii(key);
  if (ascii == 'a' || ascii == 'A') {
    applyMissionSettings();
    return true;
  }
  return true;
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
      if (context_.dropdown_mode == navigation::ui::MapDropdownMode::LoadPoints &&
        context_.dropdown_marked_order.size() >= 2)
      {
        beginMergeSelectedPointFiles();
        return;
      }
      selectDropdownOption(option_index);
    });
}

bool NavigationUiCoordinator::handleActiveInputKey(int key)
{
  if (handleTextInputKey(key)) {
    return true;
  }
  if (handleSegmentSpeedKey(key)) {
    return true;
  }
  if (handlePointGroupEditKey(key)) {
    return true;
  }
  if (handleRoutePatchKey(key)) {
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
  if (handleSettingsKey(key)) {
    return true;
  }
  return handleDropdownKey(key);
}

bool NavigationUiCoordinator::handleRoutePatchKey(int key)
{
  if (!context_.route_patch_active) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (navigation::keyboards::isEscKey(key)) {
    points_workflow_.cancelRoutePatch();
    return true;
  }
  if (navigation::keyboards::isEnterKey(key)) {
    points_workflow_.confirmRoutePatch();
    return true;
  }
  if (navigation::keyboards::isBackspaceKey(key)) {
    points_workflow_.removeLastRoutePatchPoint();
    return true;
  }

  context_.status_message = "Route patch: left click add, Enter confirm, Esc restore";
  return true;
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
  ui_state.dropdown_marked_indices = context_.dropdown_marked_indices;
  ui_state.panel_scroll_px = context_.panel_scroll_px;
  ui_state.panel_collapsed = context_.panel_collapsed;
  ui_state.fullscreen = context_.fullscreen;
  ui_state.mission_plan_display_mode = context_.mission_plan_display_mode;
  ui_state.core_connected = !context_.remote_control || context_.core_connected;
  ui_state.cmd_vel_valid = context_.cmd_vel_valid;
  ui_state.cmd_vel_linear_x = context_.cmd_vel_linear_x;
  ui_state.cmd_vel_linear_y = context_.cmd_vel_linear_y;
  ui_state.cmd_vel_angular_z = context_.cmd_vel_angular_z;
  ui_state.settings_active = context_.settings_popup_active;
  ui_state.settings_editing = context_.settings_editing;
  ui_state.settings_selected_index = context_.settings_selected_index;
  ui_state.settings_edit_text = context_.settings_edit_text;
  ui_state.settings_field_names = settingsFieldNames();
  ui_state.settings_field_values = settingsFieldValues();
  ui_state.segment_speed_active = context_.segment_speed_edit_active;
  ui_state.segment_speed_editing = context_.segment_speed_field_editing;
  ui_state.segment_speed_selected_index = context_.segment_speed_selected_index;
  ui_state.segment_speed_edit_text = context_.segment_speed_edit_text;
  ui_state.segment_speed_title = context_.segment_speed_title;
  ui_state.segment_speed_field_names = segmentSpeedFieldNames();
  ui_state.segment_speed_field_values = segmentSpeedFieldValues();
  ui_state.mission_plan_summary = context_.mission_plan_summary;
  ui_state.mission_plan_points = context_.mission_plan_points;
  ui_state.route_patch_active = context_.route_patch_active;
  ui_state.route_patch_insert_index = context_.route_patch_insert_index;
  ui_state.route_patch_points.clear();
  ui_state.route_patch_points.reserve(context_.route_patch_points.size());
  for (const auto & point : context_.route_patch_points) {
    ui_state.route_patch_points.push_back({point.x, point.y, false});
  }
  ui_state.point_group_select_mode_active = context_.point_group_select_mode_active;
  ui_state.point_group_selection_drag_active = context_.point_group_selection_drag_active;
  ui_state.point_group_edit_active = context_.point_group_edit_active;
  ui_state.point_group_selection_drag_start_x = context_.point_group_selection_drag_start_x;
  ui_state.point_group_selection_drag_start_y = context_.point_group_selection_drag_start_y;
  ui_state.point_group_selection_drag_end_x = context_.point_group_selection_drag_end_x;
  ui_state.point_group_selection_drag_end_y = context_.point_group_selection_drag_end_y;
  ui_state.point_group_selection_min_x = context_.point_group_selection_min_x;
  ui_state.point_group_selection_max_x = context_.point_group_selection_max_x;
  ui_state.point_group_selection_min_y = context_.point_group_selection_min_y;
  ui_state.point_group_selection_max_y = context_.point_group_selection_max_y;
  ui_state.point_group_selection_center_x = context_.point_group_selection_center_x;
  ui_state.point_group_selection_center_y = context_.point_group_selection_center_y;
  ui_state.point_group_selected_indices = context_.point_group_selected_indices;
  return ui_state;
}

std::vector<std::string> NavigationUiCoordinator::settingsFieldNames() const
{
  return {
    "Slot categories",
    "High score category",
    "High score priority",
    "Cost budget",
    "Alpha",
    "Beta",
    "Eta",
    "Pick/place g",
    "Storage near",
    "Storage far",
    "Return near",
    "Return far",
  };
}

std::vector<std::string> NavigationUiCoordinator::settingsFieldValues() const
{
  return {
    context_.mission_slot_categories_text,
    context_.mission_high_score_category_text,
    context_.mission_high_score_priority_text,
    context_.mission_cost_budget_text,
    context_.mission_alpha_text,
    context_.mission_beta_text,
    context_.mission_eta_text,
    context_.mission_g_pick_place_text,
    context_.mission_storage_near_distance_text,
    context_.mission_storage_far_distance_text,
    context_.mission_return_near_distance_text,
    context_.mission_return_far_distance_text,
  };
}

std::vector<std::string> NavigationUiCoordinator::segmentSpeedFieldNames() const
{
  return {
    "Mode",
    "Level",
    context_.segment_speed_constant_mode ? "linear_x" : "max_linear_speed",
    "max_angular_speed",
    "k_alpha",
    "k_beta",
  };
}

std::vector<std::string> NavigationUiCoordinator::segmentSpeedFieldValues() const
{
  return {
    context_.segment_speed_constant_mode ? "Constant speed" : "P control",
    std::to_string(static_cast<int>(context_.segment_speed_level)),
    navigation::ui::formatDouble(context_.segment_speed_linear_x),
    navigation::ui::formatDouble(context_.segment_speed_max_angular_speed),
    navigation::ui::formatDouble(context_.segment_speed_k_alpha),
    navigation::ui::formatDouble(context_.segment_speed_k_beta),
  };
}

void NavigationUiCoordinator::closeSegmentSpeedEditor()
{
  context_.segment_speed_edit_active = false;
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
  context_.status_message = "Segment speed editor closed";
}

void NavigationUiCoordinator::selectSegmentSpeedField(int field_index)
{
  const auto names = segmentSpeedFieldNames();
  if (field_index < 0 || field_index >= static_cast<int>(names.size())) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool double_click = field_index == context_.segment_speed_last_click_index &&
    (now - context_.segment_speed_last_click_time) <= std::chrono::milliseconds(450);

  context_.segment_speed_selected_index = field_index;
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
  context_.segment_speed_last_click_index = field_index;
  context_.segment_speed_last_click_time = now;

  if (!double_click) {
    context_.status_message = "Segment speed field selected";
    return;
  }

  if (field_index == 0) {
    context_.segment_speed_constant_mode = !context_.segment_speed_constant_mode;
    context_.status_message = context_.segment_speed_constant_mode ?
      "Segment mode: constant speed" :
      "Segment mode: P control";
    return;
  }

  beginSegmentSpeedFieldEdit();
}

void NavigationUiCoordinator::beginSegmentSpeedFieldEdit()
{
  const auto values = segmentSpeedFieldValues();
  if (context_.segment_speed_selected_index < 0 ||
    context_.segment_speed_selected_index >= static_cast<int>(values.size()))
  {
    return;
  }
  context_.segment_speed_field_editing = true;
  context_.segment_speed_edit_text = values[static_cast<std::size_t>(context_.segment_speed_selected_index)];
  context_.status_message = "Typing segment speed field";
}

void NavigationUiCoordinator::applySegmentSpeedFieldEdit()
{
  const int selected = context_.segment_speed_selected_index;
  std::string error;
  if (selected == 0) {
    context_.segment_speed_field_editing = false;
    context_.segment_speed_edit_text.clear();
    return;
  }

  if (selected == 1) {
    int level = 0;
    if (!parseIntField(context_.segment_speed_edit_text, "Level", level, error) || level < 1 || level > 7) {
      context_.status_message = error.empty() ? "Level must be 1..7" : error;
      return;
    }
    context_.segment_speed_level = static_cast<std::uint8_t>(level);
    context_.segment_speed_field_editing = false;
    context_.segment_speed_edit_text.clear();
    applySegmentSpeedLevelDefaults();
    return;
  }

  double value = 0.0;
  const auto names = segmentSpeedFieldNames();
  const std::string label =
    selected >= 0 && selected < static_cast<int>(names.size()) ? names[static_cast<std::size_t>(selected)] : "Value";
  const bool positive_field = selected == 2 || selected == 3;
  const bool parsed = positive_field ?
    parseDoubleField(context_.segment_speed_edit_text, label, value, error) :
    parseSignedDoubleField(context_.segment_speed_edit_text, label, value, error);
  if (!parsed || (positive_field && value <= 0.0)) {
    context_.status_message = error.empty() ? label + " must be > 0" : error;
    return;
  }

  if (selected == 2) {
    context_.segment_speed_linear_x = value;
  } else if (selected == 3) {
    context_.segment_speed_max_angular_speed = value;
  } else if (selected == 4) {
    context_.segment_speed_k_alpha = value;
  } else if (selected == 5) {
    context_.segment_speed_k_beta = value;
  }
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
  context_.status_message = "Segment field updated";
}

void NavigationUiCoordinator::applySegmentSpeedLevelDefaults()
{
  context_.segment_speed_linear_x =
    context_.controller_config.constant_speed_linear_x *
    segmentSpeedFactorForLevel(context_.segment_speed_level);
  context_.segment_speed_max_angular_speed = context_.controller_config.max_angular_speed;
  context_.segment_speed_k_alpha = context_.controller_config.k_alpha;
  context_.segment_speed_k_beta = context_.controller_config.k_beta;
  context_.status_message = "Segment level defaults loaded";
}

void NavigationUiCoordinator::applySegmentSpeed()
{
  if (!context_.segment_speed_edit_active) {
    return;
  }
  points_workflow_.setSegmentSpeedValues(
    context_.segment_speed_edit_target_index,
    true,
    context_.segment_speed_constant_mode,
    context_.segment_speed_level,
    context_.segment_speed_linear_x,
    context_.segment_speed_max_angular_speed,
    context_.segment_speed_k_alpha,
    context_.segment_speed_k_beta);
  context_.segment_speed_edit_active = false;
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
}

void NavigationUiCoordinator::clearSegmentSpeed()
{
  if (!context_.segment_speed_edit_active) {
    return;
  }
  points_workflow_.setSegmentSpeedValues(
    context_.segment_speed_edit_target_index,
    false,
    true,
    0,
    0.0,
    0.0,
    0.0,
    0.0);
  context_.segment_speed_edit_active = false;
  context_.segment_speed_field_editing = false;
  context_.segment_speed_edit_text.clear();
}

void NavigationUiCoordinator::openSettings()
{
  clearDropdown();
  resetTextInput();
  context_.params_session.setActive(false);
  context_.radar_popup_active = false;
  context_.segment_speed_edit_active = false;
  context_.segment_speed_field_editing = false;
  context_.settings_popup_active = true;
  context_.settings_editing = false;
  context_.settings_edit_text.clear();
  context_.settings_selected_index = 0;
  context_.status_message = "Task settings";
}

void NavigationUiCoordinator::closeSettings()
{
  context_.settings_popup_active = false;
  context_.settings_editing = false;
  context_.settings_edit_text.clear();
  context_.status_message = "Settings closed";
}

void NavigationUiCoordinator::selectSettingsField(int field_index)
{
  const auto names = settingsFieldNames();
  if (field_index < 0 || field_index >= static_cast<int>(names.size())) {
    return;
  }
  context_.settings_selected_index = field_index;
  context_.settings_editing = false;
  context_.settings_edit_text.clear();
  beginSettingsEdit();
}

void NavigationUiCoordinator::beginSettingsEdit()
{
  const auto values = settingsFieldValues();
  if (context_.settings_selected_index < 0 ||
    context_.settings_selected_index >= static_cast<int>(values.size()))
  {
    return;
  }
  context_.settings_editing = true;
  context_.settings_edit_text = values[static_cast<std::size_t>(context_.settings_selected_index)];
  context_.status_message = "Typing setting";
}

void NavigationUiCoordinator::applySettingsEdit()
{
  const auto value = context_.settings_edit_text;
  switch (context_.settings_selected_index) {
    case 0:
      context_.mission_slot_categories_text = value;
      break;
    case 1:
      context_.mission_high_score_category_text = value;
      break;
    case 2:
      context_.mission_high_score_priority_text = value;
      break;
    case 3:
      context_.mission_cost_budget_text = value;
      break;
    case 4:
      context_.mission_alpha_text = value;
      break;
    case 5:
      context_.mission_beta_text = value;
      break;
    case 6:
      context_.mission_eta_text = value;
      break;
    case 7:
      context_.mission_g_pick_place_text = value;
      break;
    case 8:
      context_.mission_storage_near_distance_text = value;
      break;
    case 9:
      context_.mission_storage_far_distance_text = value;
      break;
    case 10:
      context_.mission_return_near_distance_text = value;
      break;
    case 11:
      context_.mission_return_far_distance_text = value;
      break;
    default:
      break;
  }
  context_.settings_editing = false;
  context_.settings_edit_text.clear();
  context_.status_message = "Setting applied";
}

void NavigationUiCoordinator::applyMissionSettings()
{
  if (context_.settings_editing) {
    applySettingsEdit();
  }

  std::string error;
  std::vector<int> slot_categories;
  if (!parseSlotCategories(context_.mission_slot_categories_text, slot_categories, error)) {
    context_.status_message = error;
    return;
  }

  std::optional<int> high_score_category;
  if (!parseHighScoreCategory(context_.mission_high_score_category_text, high_score_category, error)) {
    context_.status_message = error;
    return;
  }

  int high_score_priority = 0;
  if (!parseIntField(context_.mission_high_score_priority_text, "High score priority", high_score_priority, error)) {
    context_.status_message = error;
    return;
  }
  if (high_score_priority != 0 && high_score_priority != 1) {
    context_.status_message = "High score priority must be 0 or 1";
    return;
  }

  double cost_budget = 0.0;
  double alpha = 0.0;
  double beta = 0.0;
  double eta = 0.0;
  double g_pick_place = 0.0;
  double storage_near_distance = 0.0;
  double storage_far_distance = 0.0;
  double return_near_distance = 0.0;
  double return_far_distance = 0.0;
  if (!parseDoubleField(context_.mission_cost_budget_text, "Cost budget", cost_budget, error) ||
    !parseDoubleField(context_.mission_alpha_text, "Alpha", alpha, error) ||
    !parseDoubleField(context_.mission_beta_text, "Beta", beta, error) ||
    !parseDoubleField(context_.mission_eta_text, "Eta", eta, error) ||
    !parseDoubleField(context_.mission_g_pick_place_text, "Pick/place g", g_pick_place, error) ||
    !parseDoubleField(
      context_.mission_storage_near_distance_text,
      "Storage near",
      storage_near_distance,
      error) ||
    !parseDoubleField(
      context_.mission_storage_far_distance_text,
      "Storage far",
      storage_far_distance,
      error) ||
    !parseDoubleField(
      context_.mission_return_near_distance_text,
      "Return near",
      return_near_distance,
      error) ||
    !parseDoubleField(
      context_.mission_return_far_distance_text,
      "Return far",
      return_far_distance,
      error))
  {
    context_.status_message = error;
    return;
  }
  if (storage_near_distance <= 0.0 || storage_far_distance <= 0.0 ||
    return_near_distance <= 0.0 || return_far_distance <= 0.0)
  {
    context_.status_message = "Near/Far distances must be > 0";
    return;
  }
  if (storage_far_distance < storage_near_distance) {
    context_.status_message = "Storage far must be >= storage near";
    return;
  }
  if (return_far_distance < return_near_distance) {
    context_.status_message = "Return far must be >= return near";
    return;
  }

  try {
    navigation::optim::PlanningRequest request;
    request.slot_category = slot_categories;
    request.high_score_category = high_score_category;
    request.high_score_priority = high_score_priority == 1;
    request.cost_budget = cost_budget;
    request.alpha = alpha;
    request.beta = beta;
    request.eta = eta;
    request.g_pick_place = g_pick_place;

    const auto result = navigation::optim::planTaskOrder(request);
    const auto generated_points = buildMissionNavigationRoute(
      result,
      storage_near_distance,
      storage_far_distance,
      return_near_distance,
      return_far_distance);
    context_.mission_plan_points.clear();
    if (!result.order.empty()) {
      context_.mission_plan_points.push_back({result.start_position.x, result.start_position.y, false});
      for (const auto & step : result.steps) {
        context_.mission_plan_points.push_back({step.box_position.x, step.box_position.y, false});
        context_.mission_plan_points.push_back({step.return_zone_position.x, step.return_zone_position.y, true});
      }
    }
    if (!generated_points.empty()) {
      runtime_.stopNavigationForRouteChange();
      context_.race_logic = "mission";
      context_.map->setPoints(generated_points);
      runtime_.syncControllerWaypoints();
    }

    std::ostringstream summary;
    summary << "Plan ";
    if (result.order.empty()) {
      summary << "empty";
    } else {
      summary << formatPlanOrder(result.order);
    }
    summary << "  score=" << result.best_score << "  cost=" << std::fixed << std::setprecision(2) <<
      result.best_cost << "/" << cost_budget;
    if (!generated_points.empty()) {
      summary << "  points=" << generated_points.size();
    }
    context_.mission_plan_summary = summary.str();
    context_.status_message = context_.mission_plan_summary;
    context_.settings_popup_active = false;
    context_.settings_editing = false;
    context_.settings_edit_text.clear();
  } catch (const std::exception & exc) {
    context_.status_message = std::string("Planning failed: ") + exc.what();
  }
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
  points_workflow_.loadPointsFromFile(context_.points_file);
  const auto points_group = navigation::maps::pointsFileGroup(context_.points_file);
  const auto points_name = std::filesystem::path(context_.points_file).filename().string();
  context_.status_message = "Loaded map: " + std::filesystem::path(scene_path).filename().string() +
    ", points: " + (points_group.empty() ? points_name : points_group + "/" + points_name);
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
  point.constant_speed = false;

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
  runtime_.applyRadarCalibrationFile(path);
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
