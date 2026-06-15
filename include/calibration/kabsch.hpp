#ifndef NAVIGATION_CALIBRATION_KABSCH_HPP_
#define NAVIGATION_CALIBRATION_KABSCH_HPP_

#include <string>
#include <vector>

#include "maps/top_view_map.hpp"

namespace navigation::calibration
{

struct KabschResult
{
  double r00{1.0};
  double r01{0.0};
  double r10{0.0};
  double r11{1.0};
  double tx{0.0};
  double ty{0.0};
  double yaw_offset{0.0};
  double mean_error{0.0};
  double max_error{0.0};
  bool unstable{false};
  std::vector<double> errors;
};

bool computeKabsch(
  const std::vector<navigation::maps::MapPoint> & radar_points,
  const std::vector<navigation::maps::MapPoint> & mujoco_points,
  KabschResult & result,
  std::string * error_message);

}  // namespace navigation::calibration

#endif  // NAVIGATION_CALIBRATION_KABSCH_HPP_
