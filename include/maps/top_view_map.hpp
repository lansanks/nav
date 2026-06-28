#ifndef NAVIGATION_MAPS_TOP_VIEW_MAP_HPP_
#define NAVIGATION_MAPS_TOP_VIEW_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "interface.hpp"
#include "opencv2/core.hpp"
#include "ui/map_ui_types.hpp"

namespace navigation::maps
{

constexpr std::uint8_t kTaskTypeNone = 0;
constexpr std::uint8_t kTaskTypePickup = 1;
constexpr std::uint8_t kTaskTypePlace = 2;

std::string resolveScenePath(const std::string & robot_name, const std::string & scene);
std::vector<std::string> listSceneFiles(const std::string & robot_name);

struct MapPoint
{
  int id{0};
  double x{0.0};
  double y{0.0};
  bool fast{false};
  std::uint8_t task_type{kTaskTypeNone};
};

using MapUiAction = navigation::ui::MapUiAction;
using MapDropdownMode = navigation::ui::MapDropdownMode;
using MapUiHit = navigation::ui::MapUiHit;
using MapUiState = navigation::ui::MapUiState;

class TopViewMap
{
public:
  TopViewMap(int width, int height, double padding);
  ~TopViewMap();

  void load(const std::string & scene_path);
  cv::Mat draw(const navigation::RobotNavigationState * state, const MapUiState & ui_state) const;
  std::size_t geomCount() const;
  MapUiHit hitTestUi(int pixel_x, int pixel_y, const MapUiState & ui_state) const;
  bool pixelToWorld(int pixel_x, int pixel_y, MapPoint & point) const;
  int hitTestPoint(int pixel_x, int pixel_y, int radius_px = 10) const;
  int nearestPointIndex(int pixel_x, int pixel_y) const;
  bool zoomAt(int pixel_x, int pixel_y, double factor);
  bool panBy(int delta_x, int delta_y);
  void setPoints(const std::vector<MapPoint> & points);
  void addPoint(const MapPoint & point);
  bool setPointFast(std::size_t index, bool fast);
  bool togglePointFast(std::size_t index);
  bool removePoint(std::size_t index);
  bool removeLastPoint();
  void clearPoints();
  const std::vector<MapPoint> & points() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation::maps

#endif  // NAVIGATION_MAPS_TOP_VIEW_MAP_HPP_
