#ifndef NAVIGATION_CONTROLLER_HPP_
#define NAVIGATION_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "interface.hpp"
#include "maps/top_view_map.hpp"

namespace navigation
{

struct ControllerConfig
{
  double waypoint_tolerance{0.20};
  double max_linear_speed{0.70};
  double max_angular_speed{1.80};
  double k_rho{1.20};
  double k_alpha{2.40};
  double k_beta{-0.60};
  double fast_max_linear_speed{1.20};
  double fast_max_angular_speed{2.20};
  double fast_k_rho{1.60};
  double fast_k_alpha{2.80};
  double fast_k_beta{-0.70};
};

struct ControllerStatus
{
  bool active{false};
  bool complete{false};
  std::size_t target_index{0};
  std::size_t point_count{0};
  std::string message;
};

class NavigationController
{
public:
  virtual ~NavigationController() = default;

  virtual std::string name() const = 0;
  virtual void configure(const ControllerConfig & config) = 0;
  virtual void setWaypoints(const std::vector<maps::MapPoint> & points) = 0;
  virtual bool start(
    std::string * error_message,
    const RobotNavigationState * initial_state = nullptr) = 0;
  virtual void stop() = 0;
  virtual geometry_msgs::msg::Twist update(const RobotNavigationState & state) = 0;
  virtual ControllerStatus status() const = 0;
};

std::vector<std::string> controllerNames();
std::unique_ptr<NavigationController> createController(
  const std::string & name,
  const ControllerConfig & config);

}  // namespace navigation

#endif  // NAVIGATION_CONTROLLER_HPP_
