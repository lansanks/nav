#ifndef NAVIGATION_APP_NAVIGATION_POINTS_WORKFLOW_HPP_
#define NAVIGATION_APP_NAVIGATION_POINTS_WORKFLOW_HPP_

#include <cstddef>
#include <string>

#include "app/navigation_node_context.hpp"
#include "app/navigation_runtime.hpp"
#include "rclcpp/rclcpp.hpp"

namespace navigation::app
{

class NavigationPointsWorkflow
{
public:
  NavigationPointsWorkflow(
    NavigationNodeContext & context,
    NavigationRuntime & runtime,
    rclcpp::Logger logger);

  bool savePoints();
  void loadPointsFromFile(const std::string & path_or_name);
  void savePointsAs(const std::string & path_or_name);
  void createNewPointsFile(const std::string & path_or_name);
  void mergePointsFilesAs(
    const std::vector<std::string> & source_paths,
    const std::string & path_or_name);
  void addClickedPoint(int pixel_x, int pixel_y);
  void toggleFastMarker(std::size_t index);
  void setEventLabel(std::size_t index, const std::string & event_label);
  void setSegmentSpeed(std::size_t target_index, const std::string & input_text);
  void removeNearestPoint(int pixel_x, int pixel_y);
  void confirmRoutePatch();
  void cancelRoutePatch();
  void removeLastRoutePatchPoint();
  void removeLastPoint();
  void clearPoints();

private:
  NavigationNodeContext & context_;
  NavigationRuntime & runtime_;
  rclcpp::Logger logger_;
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_POINTS_WORKFLOW_HPP_
