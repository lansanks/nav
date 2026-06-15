#ifndef NAVIGATION_MAPS_NAVIGATION_MAP_HELPERS_HPP_
#define NAVIGATION_MAPS_NAVIGATION_MAP_HELPERS_HPP_

#include <string>
#include <vector>

#include "maps/top_view_map.hpp"

namespace navigation::maps
{

int findPathIndex(const std::vector<std::string> & paths, const std::string & path);
void addExistingPath(std::vector<std::string> & paths, const std::string & path);
std::vector<std::string> makePathLabels(const std::vector<std::string> & paths);
std::string defaultNewPointsName();
bool validateFastMarkers(
  const std::vector<MapPoint> & points,
  std::string * error_message = nullptr);
int pendingFastMarkerIndex(const std::vector<MapPoint> & points);

}  // namespace navigation::maps

#endif  // NAVIGATION_MAPS_NAVIGATION_MAP_HELPERS_HPP_
