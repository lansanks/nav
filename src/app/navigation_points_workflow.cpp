#include "app/navigation_points_workflow.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "maps/navigation_map_helpers.hpp"
#include "maps/point_store.hpp"

namespace navigation::app
{
namespace
{

void renumberPoints(std::vector<navigation::maps::MapPoint> & points)
{
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].id = static_cast<int>(i + 1);
  }
}

void clearRoutePatchState(NavigationNodeContext & context)
{
  context.route_patch_active = false;
  context.route_patch_insert_index = 0;
  context.route_patch_original_points.clear();
  context.route_patch_points.clear();
}

}  // namespace


NavigationPointsWorkflow::NavigationPointsWorkflow(
  NavigationNodeContext & context,
  NavigationRuntime & runtime,
  rclcpp::Logger logger)
: context_(context),
  runtime_(runtime),
  logger_(logger)
{
}

bool NavigationPointsWorkflow::savePoints()
{
  std::string marker_error;
  if (context_.race_logic == "obstacle" &&
    !navigation::maps::validateFastMarkers(context_.map->points(), &marker_error))
  {
    context_.status_message = marker_error;
    RCLCPP_WARN(logger_, "%s", marker_error.c_str());
    return false;
  }

  std::string error;
  if (!navigation::maps::savePointsFile(context_.points_file, context_.map->points(), &error)) {
    context_.status_message = "Save failed";
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return false;
  }

  return true;
}

void NavigationPointsWorkflow::loadPointsFromFile(const std::string & path_or_name)
{
  runtime_.stopNavigationForRouteChange();
  const auto path = navigation::maps::resolvePointsFilePath(path_or_name);
  auto loaded_points = navigation::maps::loadPointsFile(path);
  context_.map->setPoints(loaded_points);
  context_.points_file = path;
  runtime_.syncControllerWaypoints();
  RCLCPP_INFO(
    logger_,
    "Loaded %zu navigation points from %s",
    loaded_points.size(),
    context_.points_file.c_str());
}

void NavigationPointsWorkflow::savePointsAs(const std::string & path_or_name)
{
  std::string marker_error;
  if (context_.race_logic == "obstacle" &&
    !navigation::maps::validateFastMarkers(context_.map->points(), &marker_error))
  {
    context_.status_message = marker_error;
    RCLCPP_WARN(logger_, "%s", marker_error.c_str());
    return;
  }

  context_.points_file = navigation::maps::resolvePointsFilePath(path_or_name);
  if (savePoints()) {
    context_.status_message = "Saved: " + std::filesystem::path(context_.points_file).filename().string();
    RCLCPP_INFO(logger_, "Saved navigation points as %s", context_.points_file.c_str());
  }
}

void NavigationPointsWorkflow::createNewPointsFile(const std::string & path_or_name)
{
  runtime_.stopNavigationForRouteChange();
  const auto path = navigation::maps::resolvePointsFilePath(path_or_name);
  std::error_code filesystem_error;
  if (std::filesystem::exists(path, filesystem_error)) {
    context_.status_message = "Point file already exists";
    RCLCPP_WARN(logger_, "Point file already exists: %s", path.c_str());
    return;
  }

  std::string save_error;
  const std::vector<navigation::maps::MapPoint> empty_points;
  if (!navigation::maps::savePointsFile(path, empty_points, &save_error)) {
    context_.status_message = "Create failed";
    RCLCPP_ERROR(logger_, "%s", save_error.c_str());
    return;
  }

  context_.map->clearPoints();
  context_.points_file = path;
  runtime_.syncControllerWaypoints();
  RCLCPP_INFO(logger_, "Created empty navigation points file: %s", context_.points_file.c_str());
}

void NavigationPointsWorkflow::mergePointsFilesAs(
  const std::vector<std::string> & source_paths,
  const std::string & path_or_name)
{
  if (source_paths.size() < 2) {
    context_.status_message = "Select at least two point files";
    return;
  }

  std::vector<navigation::maps::MapPoint> merged_points;
  for (const auto & source_path : source_paths) {
    auto points = navigation::maps::loadPointsFile(source_path);
    if (points.empty()) {
      context_.status_message = "Merge failed: empty point file";
      RCLCPP_WARN(logger_, "Cannot merge empty point file: %s", source_path.c_str());
      return;
    }
    merged_points.insert(merged_points.end(), points.begin(), points.end());
  }

  renumberPoints(merged_points);
  std::string marker_error;
  if (context_.race_logic == "obstacle" &&
    !navigation::maps::validateFastMarkers(merged_points, &marker_error))
  {
    context_.status_message = marker_error;
    RCLCPP_WARN(logger_, "Merged point route rejected: %s", marker_error.c_str());
    return;
  }

  const auto output_path = navigation::maps::resolvePointsFilePath(path_or_name);
  std::string error;
  if (!navigation::maps::savePointsFile(output_path, merged_points, &error)) {
    context_.status_message = "Merge save failed";
    RCLCPP_ERROR(logger_, "Failed to save merged point file: %s", error.c_str());
    return;
  }

  runtime_.stopNavigationForRouteChange();
  context_.points_file = output_path;
  context_.map->setPoints(merged_points);
  runtime_.syncControllerWaypoints();
  context_.status_message =
    "Merged points: " + std::filesystem::path(context_.points_file).filename().string();
  RCLCPP_INFO(
    logger_,
    "Merged %zu point files into %s with %zu points.",
    source_paths.size(),
    context_.points_file.c_str(),
    merged_points.size());
}

void NavigationPointsWorkflow::addClickedPoint(int pixel_x, int pixel_y)
{
  if (context_.route_patch_active) {
    navigation::maps::MapPoint point;
    if (!context_.map->pixelToWorld(pixel_x, pixel_y, point)) {
      RCLCPP_WARN(logger_, "Click is outside the map bounds.");
      return;
    }

    point.fast = false;
    point.constant_speed = false;
    point.task_type = navigation::maps::kTaskTypeNone;
    point.event_label.clear();
    context_.route_patch_points.push_back(point);
    context_.status_message = "Patch point added. Enter confirm, Esc restore";
    return;
  }

  const int point_index = context_.map->hitTestPoint(pixel_x, pixel_y);
  if (point_index >= 0) {
    toggleFastMarker(static_cast<std::size_t>(point_index));
    return;
  }

  navigation::maps::MapPoint point;
  if (!context_.map->pixelToWorld(pixel_x, pixel_y, point)) {
    RCLCPP_WARN(logger_, "Click is outside the map bounds.");
    return;
  }

  runtime_.stopNavigationForRouteChange();
  point.id = static_cast<int>(context_.map->points().size() + 1);
  point.fast = false;
  point.constant_speed = false;
  context_.map->addPoint(point);
  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    RCLCPP_INFO(
      logger_,
      "Saved point %d: x=%.3f y=%.3f fast=%s",
      point.id,
      point.x,
      point.y,
      point.fast ? "true" : "false");
  }
}

void NavigationPointsWorkflow::setEventLabel(std::size_t index, const std::string & event_label)
{
  if (index >= context_.map->points().size()) {
    return;
  }

  runtime_.stopNavigationForRouteChange();
  context_.map->setPointEventLabel(index, event_label);
  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    const auto point_id = context_.map->points()[index].id;
    context_.status_message = event_label.empty() ? "Event marker cleared" : "Event marker set: " + event_label;
    RCLCPP_INFO(
      logger_,
      "Point %d event_label set to '%s'.",
      point_id,
      event_label.c_str());
  }
}

void NavigationPointsWorkflow::toggleFastMarker(std::size_t index)
{
  const auto & points = context_.map->points();
  if (index >= points.size()) {
    return;
  }

  runtime_.stopNavigationForRouteChange();
  if (context_.race_logic == "mission") {
    const bool was_fast = points[index].fast;
    context_.map->setPointFast(index, !points[index].fast);
    runtime_.syncControllerWaypoints();
    savePoints();
    context_.status_message = was_fast ? "Mission point cleared" : "Mission point marked";
    return;
  }

  if (points[index].constant_speed) {
    context_.map->setPointConstantSpeed(index, false);
    runtime_.syncControllerWaypoints();
    if (savePoints()) {
      context_.status_message = "Point marker cleared";
    }
    return;
  }

  if (points[index].fast) {
    context_.map->setPointFast(index, false);
    context_.map->setPointConstantSpeed(index, true);
    runtime_.syncControllerWaypoints();
    if (savePoints()) {
      context_.status_message = "Blue constant-speed pair marked";
    }
    return;
  }

  context_.map->setPointFast(index, true);
  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    context_.status_message = "Fast red segment marked";
  }
}

void NavigationPointsWorkflow::removeNearestPoint(int pixel_x, int pixel_y)
{
  if (context_.route_patch_active) {
    context_.status_message = "Finish route patch first";
    return;
  }

  const int point_index = context_.map->nearestPointIndex(pixel_x, pixel_y);
  if (point_index < 0) {
    return;
  }

  const auto index = static_cast<std::size_t>(point_index);
  const auto & points = context_.map->points();
  const int removed_id = points[index].id;
  const bool patch_middle_point = index > 0 && index + 1 < points.size();

  runtime_.stopNavigationForRouteChange();
  if (patch_middle_point) {
    context_.route_patch_active = true;
    context_.route_patch_insert_index = index;
    context_.route_patch_original_points = points;
    context_.route_patch_points.clear();
  }

  if (!context_.map->removePoint(index)) {
    clearRoutePatchState(context_);
    return;
  }

  if (patch_middle_point) {
    context_.status_message = "Patch route: add points, Enter confirm, Esc restore";
    RCLCPP_INFO(
      logger_,
      "Removed middle navigation point %d. Waiting for route patch points.",
      removed_id);
    return;
  }

  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    RCLCPP_INFO(
      logger_,
      "Removed nearest navigation point %d. Remaining points: %zu",
      removed_id,
      context_.map->points().size());
  }
}

void NavigationPointsWorkflow::confirmRoutePatch()
{
  if (!context_.route_patch_active) {
    return;
  }

  auto points = context_.map->points();
  const auto insert_index = std::min(context_.route_patch_insert_index, points.size());
  for (auto & point : context_.route_patch_points) {
    point.fast = false;
    point.constant_speed = false;
    point.task_type = navigation::maps::kTaskTypeNone;
    point.event_label.clear();
  }
  points.insert(
    points.begin() + static_cast<std::ptrdiff_t>(insert_index),
    context_.route_patch_points.begin(),
    context_.route_patch_points.end());
  renumberPoints(points);
  context_.map->setPoints(points);
  clearRoutePatchState(context_);

  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    context_.status_message = "Route patch applied";
    RCLCPP_INFO(logger_, "Route patch applied. Total points: %zu", context_.map->points().size());
  }
}

void NavigationPointsWorkflow::cancelRoutePatch()
{
  if (!context_.route_patch_active) {
    return;
  }

  context_.map->setPoints(context_.route_patch_original_points);
  clearRoutePatchState(context_);
  runtime_.syncControllerWaypoints();
  context_.status_message = "Route patch cancelled";
  RCLCPP_INFO(logger_, "Route patch cancelled.");
}

void NavigationPointsWorkflow::removeLastRoutePatchPoint()
{
  if (!context_.route_patch_active) {
    return;
  }

  if (context_.route_patch_points.empty()) {
    context_.status_message = "No patch points to remove";
    return;
  }

  context_.route_patch_points.pop_back();
  context_.status_message = "Patch point removed";
}

void NavigationPointsWorkflow::removeLastPoint()
{
  if (!context_.map->removeLastPoint()) {
    return;
  }

  runtime_.stopNavigationForRouteChange();
  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    RCLCPP_INFO(logger_, "Removed last navigation point. Remaining points: %zu", context_.map->points().size());
  }
}

void NavigationPointsWorkflow::clearPoints()
{
  if (context_.route_patch_active) {
    cancelRoutePatch();
  }

  if (context_.map->points().empty()) {
    return;
  }

  runtime_.stopNavigationForRouteChange();
  context_.map->clearPoints();
  runtime_.syncControllerWaypoints();
  if (savePoints()) {
    RCLCPP_INFO(logger_, "Cleared navigation points.");
  }
}

}  // namespace navigation::app
