#ifndef NAVIGATION_CALIBRATION_RADAR_CALIBRATION_HPP_
#define NAVIGATION_CALIBRATION_RADAR_CALIBRATION_HPP_

#include <string>
#include <vector>

#include "calibration/kabsch.hpp"
#include "maps/top_view_map.hpp"

namespace navigation::calibration
{

std::string defaultCalibrationDataDir();
std::string defaultCalibrationParamsDir();
std::string resolveCalibrationDataFilePath(const std::string & path_or_name);
std::string resolveCalibrationParamsFilePath(
  const std::string & radar_points_file,
  const std::string & mujoco_points_file);
std::vector<std::string> listCalibrationDataFiles();
bool appendRadarPoint(
  const std::string & path_or_name,
  const navigation::maps::MapPoint & point,
  std::string * saved_path,
  std::string * error_message);
bool saveCalibrationParams(
  const std::string & path,
  const std::string & radar_points_file,
  const std::string & mujoco_points_file,
  const KabschResult & result,
  std::string * error_message);

}  // namespace navigation::calibration

#endif  // NAVIGATION_CALIBRATION_RADAR_CALIBRATION_HPP_
