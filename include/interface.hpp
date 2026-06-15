#ifndef NAVIGATION_INTERFACE_HPP_
#define NAVIGATION_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace navigation
{

struct RobotNavigationState
{
  bool valid{false};
  rclcpp::Time stamp{};
  std::string frame_id;
  std::string source;

  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};

  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_z{0.0};
  double planar_speed{0.0};
};

class NavigationInterface
{
public:
  virtual ~NavigationInterface() = default;

  virtual void start(rclcpp::Node & node) = 0;
  virtual bool getState(RobotNavigationState & state) const = 0;
  virtual std::string sourceName() const = 0;
};

std::unique_ptr<NavigationInterface> createSimulationInterface();
std::unique_ptr<NavigationInterface> createRadarInterface();

}  // namespace navigation

#endif  // NAVIGATION_INTERFACE_HPP_
