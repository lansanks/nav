#ifndef NAVIGATION_OPTIM_TASK_ORDER_PLANNER_HPP_
#define NAVIGATION_OPTIM_TASK_ORDER_PLANNER_HPP_

#include <optional>
#include <string>
#include <vector>

namespace navigation::optim
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct PlanningStep
{
  int slot_id{0};
  int category{0};
  std::string category_name;
  int target_zone_id{0};
  int reward{0};
  Point2 box_position;
  Point2 return_zone_position;
  double step_cost{0.0};
  double cumulative_cost{0.0};
  double empty_distance{0.0};
  double loaded_distance{0.0};
  double turn_cost{0.0};
  double fixed_cost{0.0};
};

struct PlanningMapPoint
{
  int id{0};
  Point2 position;
};

struct PlanningResult
{
  std::vector<int> order;
  int best_score{0};
  double best_cost{0.0};
  double cost_budget{0.0};
  std::vector<PlanningStep> steps;
  std::vector<int> considered_slots;
  std::optional<int> high_score_category;
  Point2 start_position;
  double start_yaw{0.0};
  std::vector<PlanningMapPoint> storage_slots;
  std::vector<PlanningMapPoint> return_zones;
};

struct PlanningRequest
{
  std::vector<int> slot_category;
  std::optional<int> high_score_category;
  double cost_budget{0.0};
  Point2 current_position{3.7, -9.45};
  double current_yaw{1.57079632679};
  std::vector<int> remaining_slots;
  double alpha{1.0};
  double beta{0.3};
  double eta{0.4};
  double g_pick_place{0.0};
  std::string geometry_path;
};

PlanningResult planTaskOrder(const PlanningRequest & request);

}  // namespace navigation::optim

#endif  // NAVIGATION_OPTIM_TASK_ORDER_PLANNER_HPP_
