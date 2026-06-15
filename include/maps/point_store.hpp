#ifndef NAVIGATION_MAPS_POINT_STORE_HPP_
#define NAVIGATION_MAPS_POINT_STORE_HPP_

#include <string>
#include <vector>

#include "maps/top_view_map.hpp"

namespace navigation::maps
{

std::string defaultPointsFilePath();
std::string resolvePointsFilePath(const std::string & path_or_name);
std::vector<std::string> listPointsFiles(const std::string & include_path = "");
std::vector<MapPoint> loadPointsFile(const std::string & path);
bool savePointsFile(const std::string & path, const std::vector<MapPoint> & points, std::string * error_message);

}  // namespace navigation::maps

#endif  // NAVIGATION_MAPS_POINT_STORE_HPP_
