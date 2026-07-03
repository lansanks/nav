#include "optim/task_order_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace navigation::optim
{
namespace
{

constexpr double kEps = 1.0e-9;
constexpr double kColumnXTolerance = 0.05;

struct PickupOption
{
  int transition_id{0};
  Point2 transition_position;
  int stop_id{0};
  Point2 stop_position;
  bool startup_reachable{false};
};

struct StartupTransition
{
  int id{0};
  Point2 position;
};

struct MapGeometry
{
  std::map<int, Point2> storage_slots;
  std::map<int, Point2> return_zones;
  std::map<int, std::string> return_zone_names;
  std::map<int, std::vector<PickupOption>> pickup_options;
  std::vector<StartupTransition> startup_transitions;
  Point2 start_position{0.0, 0.0};
  double start_yaw{1.57079632679};
};

struct LegCost
{
  double total{0.0};
  double empty_distance{0.0};
  double loaded_distance{0.0};
  double turn_cost{0.0};
  double fixed_cost{0.0};
  int startup_transition_id{0};
  Point2 startup_transition_position;
  int pickup_transition_id{0};
  Point2 pickup_transition_position;
  int pickup_stop_id{0};
  Point2 pickup_stop_position;
};

std::string trim(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

bool startsWith(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

double parseDouble(const std::string & text)
{
  return std::stod(trim(text));
}

std::filesystem::path defaultGeometryPath()
{
  auto current = std::filesystem::current_path();
  while (true) {
    for (const auto & candidate : {
        current / "config" / "points" / "tezheng.yaml",
        current / "config" / "mission" / "map_geom.yaml"})
    {
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
    if (!current.has_parent_path() || current == current.parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  const auto share_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("navigation"));
  for (auto current_share = share_dir; !current_share.empty(); current_share = current_share.parent_path()) {
    for (const auto & candidate : {
        current_share / "src" / "navigation" / "config" / "points" / "tezheng.yaml",
        current_share / "src" / "navigation" / "config" / "mission" / "map_geom.yaml"})
    {
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
    if (current_share == current_share.parent_path()) {
      break;
    }
  }
  const auto feature_points = share_dir / "config" / "points" / "tezheng.yaml";
  if (std::filesystem::exists(feature_points)) {
    return feature_points;
  }
  return share_dir / "config" / "mission" / "map_geom.yaml";
}

bool parseInlineCenter(const std::string & line, Point2 & point)
{
  static const std::regex center_regex(
    R"(center:\s*\{x:\s*([-+0-9.eE]+),\s*y:\s*([-+0-9.eE]+))");
  std::smatch match;
  if (!std::regex_search(line, match, center_regex)) {
    return false;
  }
  point.x = parseDouble(match[1].str());
  point.y = parseDouble(match[2].str());
  return true;
}

std::optional<int> parseListId(const std::string & line)
{
  static const std::regex id_regex(R"(^-\s*id:\s*([0-9]+))");
  std::smatch match;
  const auto text = trim(line);
  if (!std::regex_search(text, match, id_regex)) {
    return std::nullopt;
  }
  return std::stoi(match[1].str());
}

std::optional<std::string> parseName(const std::string & line)
{
  static const std::regex name_regex(R"(^name:\s*\"?([^\"\s#]+))");
  std::smatch match;
  const auto text = trim(line);
  if (!std::regex_search(text, match, name_regex)) {
    return std::nullopt;
  }
  return match[1].str();
}

bool parseScalarDouble(const std::string & text, const std::string & key, double & value)
{
  const auto prefix = key + ":";
  if (!startsWith(text, prefix)) {
    return false;
  }
  value = parseDouble(text.substr(prefix.size()));
  return true;
}

Point2 midpoint(Point2 a, Point2 b)
{
  return {0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
}

MapGeometry loadFeaturePointGeometry(
  const std::vector<std::string> & lines,
  const std::filesystem::path & path)
{
  std::map<int, Point2> points;
  int current_id = 0;
  Point2 current;
  bool in_point = false;
  bool have_x = false;
  bool have_y = false;

  auto commit_point = [&]() {
      if (!in_point || current_id <= 0 || !have_x || !have_y) {
        return;
      }
      points[current_id] = current;
    };

  for (const auto & line : lines) {
    const auto text = trim(line);
    if (text.empty() || startsWith(text, "#")) {
      continue;
    }

    if (startsWith(text, "-")) {
      commit_point();
      in_point = true;
      have_x = false;
      have_y = false;
      current = Point2{};
      current_id = 0;
      if (auto id = parseListId(text)) {
        current_id = *id;
      }
      continue;
    }

    if (!in_point) {
      continue;
    }

    if (startsWith(text, "id:")) {
      current_id = std::stoi(trim(text.substr(3)));
    } else if (parseScalarDouble(text, "x", current.x)) {
      have_x = true;
    } else if (parseScalarDouble(text, "y", current.y)) {
      have_y = true;
    }
  }
  commit_point();

  auto require_point = [&](int id) -> Point2 {
      const auto iter = points.find(id);
      if (iter == points.end()) {
        throw std::runtime_error(
          "mission feature points are missing id " + std::to_string(id) + ": " + path.string());
      }
      return iter->second;
    };

  MapGeometry geometry;
  geometry.start_position = {0.0, 0.0};
  geometry.start_yaw = 1.57079632679;

  // Keep the existing 0..3 category ids.  On task.png these map right-to-left
  // to feature points 4, 3, 2, 1, matching the old return-zone category order.
  constexpr std::array<int, 4> kReturnPointIds{4, 3, 2, 1};
  for (int category = 0; category < static_cast<int>(kReturnPointIds.size()); ++category) {
    const int point_id = kReturnPointIds[static_cast<std::size_t>(category)];
    geometry.return_zones[category] = require_point(point_id);
    geometry.return_zone_names[category] = "point_" + std::to_string(point_id);
  }

  constexpr std::array<int, 5> kTransitionIds{5, 6, 7, 8, 9};
  constexpr std::array<int, 5> kTopStopIds{18, 16, 14, 12, 10};
  constexpr std::array<int, 5> kBottomStopIds{19, 17, 15, 13, 11};

  auto add_slot = [&](
      int slot_id,
      int left_column,
      int right_column,
      const std::array<int, 5> & stop_ids,
      bool startup_reachable) {
      const int left_stop_id = stop_ids[static_cast<std::size_t>(left_column)];
      const int right_stop_id = stop_ids[static_cast<std::size_t>(right_column)];
      const auto left_stop = require_point(left_stop_id);
      const auto right_stop = require_point(right_stop_id);
      geometry.storage_slots[slot_id] = midpoint(left_stop, right_stop);
      geometry.pickup_options[slot_id] = {
        {
          kTransitionIds[static_cast<std::size_t>(left_column)],
          require_point(kTransitionIds[static_cast<std::size_t>(left_column)]),
          left_stop_id,
          left_stop,
          startup_reachable,
        },
        {
          kTransitionIds[static_cast<std::size_t>(right_column)],
          require_point(kTransitionIds[static_cast<std::size_t>(right_column)]),
          right_stop_id,
          right_stop,
          startup_reachable,
        },
      };
    };

  for (int column = 0; column < 4; ++column) {
    add_slot(column, column, column + 1, kTopStopIds, false);
    add_slot(column + 4, column, column + 1, kBottomStopIds, true);
  }

  for (const int id : {20, 21, 22}) {
    geometry.startup_transitions.push_back({id, require_point(id)});
  }
  return geometry;
}

MapGeometry loadMapGeometry(const std::string & path_text)
{
  const auto path = path_text.empty() ? defaultGeometryPath() : std::filesystem::path(path_text);
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open mission map geometry: " + path.string());
  }

  std::vector<std::string> lines;
  std::string line;
  bool has_feature_points = false;
  while (std::getline(input, line)) {
    const auto text = trim(line);
    if (text == "points:") {
      has_feature_points = true;
    }
    lines.push_back(line);
  }
  if (has_feature_points) {
    return loadFeaturePointGeometry(lines, path);
  }

  enum class Section { None, Start, Storage, Return };
  Section section = Section::None;
  MapGeometry geometry;
  std::optional<int> current_id;
  bool in_start_center = false;
  Point2 start;
  bool have_start_x = false;
  bool have_start_y = false;

  for (const auto & line : lines) {
    if (startsWith(line, "  ") && !startsWith(line, "    ")) {
      const auto top = trim(line);
      in_start_center = false;
      current_id.reset();
      if (top == "start_area:") {
        section = Section::Start;
      } else if (top == "storage_slots:") {
        section = Section::Storage;
      } else if (top == "return_zones:") {
        section = Section::Return;
      } else {
        section = Section::None;
      }
      continue;
    }

    const auto text = trim(line);
    if (text.empty() || startsWith(text, "#")) {
      continue;
    }

    if (section == Section::Start) {
      if (text == "center:") {
        in_start_center = true;
        continue;
      }
      if (in_start_center && startsWith(text, "x:")) {
        start.x = parseDouble(text.substr(2));
        have_start_x = true;
      } else if (in_start_center && startsWith(text, "y:")) {
        start.y = parseDouble(text.substr(2));
        have_start_y = true;
      } else if (in_start_center && startsWith(text, "yaw:")) {
        geometry.start_yaw = parseDouble(text.substr(4));
      } else if (have_start_x && have_start_y) {
        geometry.start_position = start;
      }
      if (have_start_x && have_start_y) {
        geometry.start_position = start;
      }
      continue;
    }

    if (section == Section::Storage || section == Section::Return) {
      if (auto id = parseListId(text)) {
        current_id = *id;
        continue;
      }
      if (!current_id) {
        continue;
      }
      if (section == Section::Return) {
        if (auto name = parseName(text)) {
          geometry.return_zone_names[*current_id] = *name;
          continue;
        }
      }
      Point2 center;
      if (parseInlineCenter(text, center)) {
        if (section == Section::Storage) {
          geometry.storage_slots[*current_id] = center;
        } else {
          geometry.return_zones[*current_id] = center;
        }
      }
    }
  }

  if (geometry.storage_slots.empty() || geometry.return_zones.empty()) {
    throw std::runtime_error("mission map geometry is missing storage slots or return zones");
  }
  return geometry;
}

double distance(Point2 a, Point2 b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

Point2 returnZoneCentroid(const MapGeometry & geometry)
{
  if (geometry.return_zones.empty()) {
    throw std::runtime_error("mission map geometry has no return zones");
  }

  Point2 center;
  for (const auto & [_, point] : geometry.return_zones) {
    center.x += point.x;
    center.y += point.y;
  }
  const double count = static_cast<double>(geometry.return_zones.size());
  center.x /= count;
  center.y /= count;
  return center;
}

std::map<int, int> deriveNearBeforeFarSlots(const MapGeometry & geometry)
{
  const Point2 zone_center = returnZoneCentroid(geometry);
  std::vector<int> slot_ids;
  slot_ids.reserve(geometry.storage_slots.size());
  for (const auto & [slot_id, _] : geometry.storage_slots) {
    slot_ids.push_back(slot_id);
  }

  std::sort(slot_ids.begin(), slot_ids.end(), [&](int left, int right) {
    const auto & left_point = geometry.storage_slots.at(left);
    const auto & right_point = geometry.storage_slots.at(right);
    if (std::abs(left_point.x - right_point.x) > kEps) {
      return left_point.x < right_point.x;
    }
    if (std::abs(left_point.y - right_point.y) > kEps) {
      return left_point.y < right_point.y;
    }
    return left < right;
  });

  std::vector<std::vector<int>> columns;
  for (const int slot_id : slot_ids) {
    const double slot_x = geometry.storage_slots.at(slot_id).x;
    bool assigned = false;
    for (auto & column : columns) {
      const double column_x = geometry.storage_slots.at(column.front()).x;
      if (std::abs(slot_x - column_x) <= kColumnXTolerance) {
        column.push_back(slot_id);
        assigned = true;
        break;
      }
    }
    if (!assigned) {
      columns.push_back({slot_id});
    }
  }

  std::map<int, int> prerequisites;
  for (const auto & column : columns) {
    if (column.size() != 2) {
      continue;
    }
    const int first = column[0];
    const int second = column[1];
    const double first_distance = distance(geometry.storage_slots.at(first), zone_center);
    const double second_distance = distance(geometry.storage_slots.at(second), zone_center);
    if (std::abs(first_distance - second_distance) <= kEps) {
      continue;
    }
    if (first_distance < second_distance) {
      prerequisites[second] = first;
    } else {
      prerequisites[first] = second;
    }
  }
  return prerequisites;
}

std::vector<int> buildActivePrerequisites(
  const std::vector<int> & active_slots,
  const MapGeometry & geometry)
{
  const auto near_before_far = deriveNearBeforeFarSlots(geometry);
  std::map<int, int> active_index_by_slot;
  for (int index = 0; index < static_cast<int>(active_slots.size()); ++index) {
    active_index_by_slot[active_slots[static_cast<std::size_t>(index)]] = index;
  }

  std::vector<int> prerequisite_indices(active_slots.size(), -1);
  for (int index = 0; index < static_cast<int>(active_slots.size()); ++index) {
    const int slot_id = active_slots[static_cast<std::size_t>(index)];
    const auto prerequisite = near_before_far.find(slot_id);
    if (prerequisite == near_before_far.end()) {
      continue;
    }
    const auto active_prerequisite = active_index_by_slot.find(prerequisite->second);
    if (active_prerequisite != active_index_by_slot.end()) {
      prerequisite_indices[static_cast<std::size_t>(index)] = active_prerequisite->second;
    }
  }
  return prerequisite_indices;
}

std::optional<double> heading(Point2 a, Point2 b)
{
  if (distance(a, b) <= kEps) {
    return std::nullopt;
  }
  return std::atan2(b.y - a.y, b.x - a.x);
}

double angleDiff(double a, double b)
{
  return std::abs(std::atan2(std::sin(b - a), std::cos(b - a)));
}

LegCost startLegCost(
  Point2 current_position,
  double current_yaw,
  Point2 box_position,
  Point2 return_zone_position,
  double alpha,
  double beta,
  double eta,
  double fixed_cost)
{
  LegCost cost;
  cost.empty_distance = distance(current_position, box_position);
  cost.loaded_distance = distance(box_position, return_zone_position);
  const auto empty_heading = heading(current_position, box_position);
  const auto loaded_heading = heading(box_position, return_zone_position);
  const double first_turn = empty_heading ? angleDiff(current_yaw, *empty_heading) : 0.0;
  const double incoming_heading = empty_heading.value_or(current_yaw);
  const double pickup_turn = loaded_heading ? angleDiff(incoming_heading, *loaded_heading) : 0.0;
  cost.turn_cost = beta * (first_turn + pickup_turn);
  cost.fixed_cost = fixed_cost;
  cost.total = alpha * cost.empty_distance + (alpha + eta) * cost.loaded_distance +
    cost.turn_cost + fixed_cost;
  cost.pickup_stop_position = box_position;
  return cost;
}

LegCost transitionLegCost(
  Point2 prev_box_position,
  Point2 prev_return_zone_position,
  Point2 next_box_position,
  Point2 next_return_zone_position,
  double alpha,
  double beta,
  double eta,
  double fixed_cost)
{
  LegCost cost;
  cost.empty_distance = distance(prev_return_zone_position, next_box_position);
  cost.loaded_distance = distance(next_box_position, next_return_zone_position);
  const auto previous_loaded_heading = heading(prev_box_position, prev_return_zone_position);
  const auto empty_heading = heading(prev_return_zone_position, next_box_position);
  const auto next_loaded_heading = heading(next_box_position, next_return_zone_position);
  const double previous_heading = previous_loaded_heading.value_or(0.0);
  const double zone_turn = empty_heading ? angleDiff(previous_heading, *empty_heading) : 0.0;
  const double incoming_heading = empty_heading.value_or(previous_heading);
  const double pickup_turn = next_loaded_heading ? angleDiff(incoming_heading, *next_loaded_heading) : 0.0;
  cost.turn_cost = beta * (zone_turn + pickup_turn);
  cost.fixed_cost = fixed_cost;
  cost.total = alpha * cost.empty_distance + (alpha + eta) * cost.loaded_distance +
    cost.turn_cost + fixed_cost;
  cost.pickup_stop_position = next_box_position;
  return cost;
}

std::vector<Point2> compactRoutePoints(const std::vector<Point2> & points)
{
  std::vector<Point2> compact;
  compact.reserve(points.size());
  for (const auto & point : points) {
    if (!compact.empty() && distance(compact.back(), point) <= kEps) {
      continue;
    }
    compact.push_back(point);
  }
  return compact;
}

double routeTurnCost(
  const std::vector<Point2> & route_points,
  std::optional<double> initial_yaw,
  double beta)
{
  if (route_points.size() < 2) {
    return 0.0;
  }

  double cost = 0.0;
  std::optional<double> previous_heading = initial_yaw;
  for (std::size_t i = 1; i < route_points.size(); ++i) {
    const auto segment_heading = heading(route_points[i - 1], route_points[i]);
    if (!segment_heading) {
      continue;
    }
    if (previous_heading) {
      cost += angleDiff(*previous_heading, *segment_heading);
    }
    previous_heading = segment_heading;
  }
  return beta * cost;
}

LegCost featureLegCost(
  Point2 current_position,
  std::optional<double> current_yaw,
  const std::optional<StartupTransition> & startup_transition,
  const PickupOption & pickup_option,
  Point2 return_zone_position,
  double alpha,
  double beta,
  double eta,
  double fixed_cost)
{
  std::vector<Point2> route_points;
  route_points.push_back(current_position);
  if (startup_transition) {
    route_points.push_back(startup_transition->position);
    route_points.push_back(pickup_option.stop_position);
    route_points.push_back(pickup_option.transition_position);
  } else {
    route_points.push_back(pickup_option.transition_position);
    route_points.push_back(pickup_option.stop_position);
    route_points.push_back(pickup_option.transition_position);
  }
  route_points.push_back(return_zone_position);
  route_points = compactRoutePoints(route_points);

  LegCost cost;
  cost.empty_distance = 0.0;
  cost.loaded_distance = 0.0;
  bool loaded_segment = false;
  for (std::size_t i = 1; i < route_points.size(); ++i) {
    const double segment_distance = distance(route_points[i - 1], route_points[i]);
    if (loaded_segment) {
      cost.loaded_distance += segment_distance;
    } else {
      cost.empty_distance += segment_distance;
    }
    if (distance(route_points[i], pickup_option.stop_position) <= kEps) {
      loaded_segment = true;
    }
  }
  cost.turn_cost = routeTurnCost(route_points, current_yaw, beta);
  cost.fixed_cost = fixed_cost;
  cost.total = alpha * cost.empty_distance + (alpha + eta) * cost.loaded_distance +
    cost.turn_cost + fixed_cost;
  if (startup_transition) {
    cost.startup_transition_id = startup_transition->id;
    cost.startup_transition_position = startup_transition->position;
  }
  cost.pickup_transition_id = pickup_option.transition_id;
  cost.pickup_transition_position = pickup_option.transition_position;
  cost.pickup_stop_id = pickup_option.stop_id;
  cost.pickup_stop_position = pickup_option.stop_position;
  return cost;
}

bool startupTransitionMatches(const StartupTransition & startup, const PickupOption & option)
{
  return option.startup_reachable &&
         std::abs(startup.position.x - option.stop_position.x) <= kColumnXTolerance;
}

LegCost bestFeatureLegCost(
  const MapGeometry & geometry,
  Point2 current_position,
  std::optional<double> current_yaw,
  int slot_id,
  Point2 return_zone_position,
  bool include_startup_transition,
  double alpha,
  double beta,
  double eta,
  double fixed_cost)
{
  const auto options = geometry.pickup_options.find(slot_id);
  if (options == geometry.pickup_options.end() || options->second.empty()) {
    throw std::runtime_error("mission feature geometry is missing pickup options for slot " + std::to_string(slot_id));
  }

  LegCost best;
  best.total = std::numeric_limits<double>::infinity();
  const bool use_startup_transition = include_startup_transition && !geometry.startup_transitions.empty();
  for (const auto & option : options->second) {
    if (use_startup_transition) {
      if (!option.startup_reachable) {
        continue;
      }
      for (const auto & startup : geometry.startup_transitions) {
        if (!startupTransitionMatches(startup, option)) {
          continue;
        }
        const auto cost = featureLegCost(
          current_position,
          current_yaw,
          startup,
          option,
          return_zone_position,
          alpha,
          beta,
          eta,
          fixed_cost);
        if (cost.total < best.total) {
          best = cost;
        }
      }
    } else {
      const auto cost = featureLegCost(
        current_position,
        current_yaw,
        std::nullopt,
        option,
        return_zone_position,
        alpha,
        beta,
        eta,
        fixed_cost);
      if (cost.total < best.total) {
        best = cost;
      }
    }
  }
  return best;
}

void validateNonNegative(double value, const char * name)
{
  if (std::isnan(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be non-negative");
  }
}

bool prerequisiteSatisfied(
  const std::vector<int> & prerequisite_indices,
  int mask,
  int index)
{
  const int prerequisite = prerequisite_indices[static_cast<std::size_t>(index)];
  return prerequisite < 0 || (mask & (1 << prerequisite)) != 0;
}

bool hasAvailableHighScoreSlot(
  int mask,
  const std::vector<int> & prerequisite_indices,
  const std::vector<bool> & high_score_slots)
{
  for (int index = 0; index < static_cast<int>(high_score_slots.size()); ++index) {
    if ((mask & (1 << index)) != 0 || !high_score_slots[static_cast<std::size_t>(index)]) {
      continue;
    }
    if (prerequisiteSatisfied(prerequisite_indices, mask, index)) {
      return true;
    }
  }
  return false;
}

bool highScorePriorityBlocksCandidate(
  int mask,
  int candidate,
  const std::vector<int> & prerequisite_indices,
  const std::vector<bool> & high_score_slots,
  bool high_score_priority)
{
  if (!high_score_priority || high_score_slots[static_cast<std::size_t>(candidate)]) {
    return false;
  }
  return hasAvailableHighScoreSlot(mask, prerequisite_indices, high_score_slots);
}

int maskBitCount(int mask)
{
  int count = 0;
  while (mask != 0) {
    mask &= mask - 1;
    ++count;
  }
  return count;
}

}  // namespace

PlanningResult planTaskOrder(const PlanningRequest & request)
{
  validateNonNegative(request.cost_budget, "cost_budget");
  validateNonNegative(request.alpha, "alpha");
  validateNonNegative(request.beta, "beta");
  validateNonNegative(request.eta, "eta");
  validateNonNegative(request.g_pick_place, "g_pick_place");
  if (request.max_steps < 0) {
    throw std::invalid_argument("max_steps must be non-negative");
  }

  const auto geometry = loadMapGeometry(request.geometry_path);
  const bool feature_geometry = !geometry.pickup_options.empty();
  const std::size_t slot_count = geometry.storage_slots.size();
  if (request.slot_category.size() != slot_count) {
    throw std::invalid_argument(
      "slot_category must contain " + std::to_string(slot_count) +
      " categories, got " + std::to_string(request.slot_category.size()));
  }

  if (request.high_score_category &&
    geometry.return_zones.find(*request.high_score_category) == geometry.return_zones.end())
  {
    throw std::invalid_argument("unknown high_score_category");
  }

  std::vector<int> active_slots = request.remaining_slots;
  if (active_slots.empty()) {
    active_slots.reserve(geometry.storage_slots.size());
    for (const auto & [slot_id, _] : geometry.storage_slots) {
      active_slots.push_back(slot_id);
    }
  }

  std::vector<int> seen_slots;
  std::vector<int> categories;
  std::vector<int> rewards;
  categories.reserve(active_slots.size());
  rewards.reserve(active_slots.size());
  for (const int slot_id : active_slots) {
    if (std::find(seen_slots.begin(), seen_slots.end(), slot_id) != seen_slots.end()) {
      throw std::invalid_argument("duplicate remaining slot id");
    }
    seen_slots.push_back(slot_id);
    if (geometry.storage_slots.find(slot_id) == geometry.storage_slots.end()) {
      throw std::invalid_argument("unknown storage slot id: " + std::to_string(slot_id));
    }
    const int category = request.slot_category.at(static_cast<std::size_t>(slot_id));
    if (geometry.return_zones.find(category) == geometry.return_zones.end()) {
      throw std::invalid_argument("slot has unknown category: " + std::to_string(category));
    }
    categories.push_back(category);
    rewards.push_back(request.high_score_category && category == *request.high_score_category ? 200 : 100);
  }

  PlanningResult result;
  result.cost_budget = request.cost_budget;
  result.considered_slots = active_slots;
  result.high_score_category = request.high_score_category;
  result.start_position = request.current_position;
  result.start_yaw = request.current_yaw;
  result.storage_slots.reserve(geometry.storage_slots.size());
  for (const auto & [slot_id, point] : geometry.storage_slots) {
    result.storage_slots.push_back({slot_id, point});
  }
  result.return_zones.reserve(geometry.return_zones.size());
  for (const auto & [zone_id, point] : geometry.return_zones) {
    result.return_zones.push_back({zone_id, point});
  }
  if (active_slots.empty()) {
    return result;
  }

  const int n = static_cast<int>(active_slots.size());
  const auto prerequisite_indices = feature_geometry ?
    std::vector<int>(active_slots.size(), -1) :
    buildActivePrerequisites(active_slots, geometry);
  const bool high_score_priority =
    request.high_score_priority && request.high_score_category.has_value();
  std::vector<bool> high_score_slots;
  high_score_slots.reserve(active_slots.size());
  for (const int category : categories) {
    high_score_slots.push_back(request.high_score_category && category == *request.high_score_category);
  }
  std::vector<LegCost> start_costs;
  start_costs.reserve(active_slots.size());
  for (int i = 0; i < n; ++i) {
    if (feature_geometry) {
      start_costs.push_back(bestFeatureLegCost(
          geometry,
          request.current_position,
          request.current_yaw,
          active_slots[i],
          geometry.return_zones.at(categories[i]),
          true,
          request.alpha,
          request.beta,
          request.eta,
          request.g_pick_place));
    } else {
      start_costs.push_back(startLegCost(
        request.current_position,
        request.current_yaw,
        geometry.storage_slots.at(active_slots[i]),
        geometry.return_zones.at(categories[i]),
        request.alpha,
        request.beta,
        request.eta,
        request.g_pick_place));
    }
  }

  std::vector<std::vector<LegCost>> transition_costs(n, std::vector<LegCost>(n));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i == j) {
        continue;
      }
      if (feature_geometry) {
        transition_costs[i][j] = bestFeatureLegCost(
          geometry,
          geometry.return_zones.at(categories[i]),
          std::nullopt,
          active_slots[j],
          geometry.return_zones.at(categories[j]),
          false,
          request.alpha,
          request.beta,
          request.eta,
          request.g_pick_place);
      } else {
        transition_costs[i][j] = transitionLegCost(
          geometry.storage_slots.at(active_slots[i]),
          geometry.return_zones.at(categories[i]),
          geometry.storage_slots.at(active_slots[j]),
          geometry.return_zones.at(categories[j]),
          request.alpha,
          request.beta,
          request.eta,
          request.g_pick_place);
      }
    }
  }

  const int total_masks = 1 << n;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> dp(total_masks, std::vector<double>(n, inf));
  std::vector<std::vector<int>> parent(total_masks, std::vector<int>(n, -1));
  const bool initial_high_score_only = high_score_priority &&
    std::any_of(high_score_slots.begin(), high_score_slots.end(), [](bool value) {return value;});
  for (int i = 0; i < n; ++i) {
    if (initial_high_score_only && !high_score_slots[static_cast<std::size_t>(i)]) {
      continue;
    }
    dp[1 << i][i] = start_costs[i].total;
  }

  for (int mask = 0; mask < total_masks; ++mask) {
    for (int last = 0; last < n; ++last) {
      const double base_cost = dp[mask][last];
      if (!std::isfinite(base_cost) || base_cost > request.cost_budget + kEps) {
        continue;
      }
      if (request.max_steps > 0 && maskBitCount(mask) >= request.max_steps) {
        continue;
      }
      for (int next = 0; next < n; ++next) {
        if ((mask & (1 << next)) != 0) {
          continue;
        }
        const int prerequisite = prerequisite_indices[static_cast<std::size_t>(next)];
        if (prerequisite >= 0 && (mask & (1 << prerequisite)) == 0) {
          continue;
        }
        if (highScorePriorityBlocksCandidate(
            mask,
            next,
            prerequisite_indices,
            high_score_slots,
            high_score_priority))
        {
          continue;
        }
        const int new_mask = mask | (1 << next);
        const double new_cost = base_cost + transition_costs[last][next].total;
        if (new_cost + kEps < dp[new_mask][next]) {
          dp[new_mask][next] = new_cost;
          parent[new_mask][next] = last;
        }
      }
    }
  }

  std::vector<int> reward_by_mask(total_masks, 0);
  for (int mask = 1; mask < total_masks; ++mask) {
    const int lsb = mask & -mask;
    int index = 0;
    while (((lsb >> index) & 1) == 0) {
      ++index;
    }
    reward_by_mask[mask] = reward_by_mask[mask ^ lsb] + rewards[index];
  }

  int best_mask = 0;
  int best_last = -1;
  for (int mask = 1; mask < total_masks; ++mask) {
    const int score = reward_by_mask[mask];
    for (int last = 0; last < n; ++last) {
      const double cost = dp[mask][last];
      if (cost > request.cost_budget + kEps) {
        continue;
      }
      if (score > result.best_score ||
        (score == result.best_score && best_last >= 0 && cost + kEps < result.best_cost))
      {
        result.best_score = score;
        result.best_cost = cost;
        best_mask = mask;
        best_last = last;
      }
    }
  }

  if (best_last < 0) {
    return result;
  }

  std::vector<int> order_indices;
  int mask = best_mask;
  int last = best_last;
  while (last >= 0) {
    order_indices.push_back(last);
    const int previous = parent[mask][last];
    mask &= ~(1 << last);
    last = previous;
  }
  std::reverse(order_indices.begin(), order_indices.end());

  double cumulative = 0.0;
  int previous_index = -1;
  for (const int index : order_indices) {
    const auto & leg_cost = previous_index < 0 ? start_costs[index] : transition_costs[previous_index][index];
    cumulative += leg_cost.total;
    const int slot_id = active_slots[index];
    const int category = categories[index];
    result.order.push_back(slot_id);
    PlanningStep step;
    step.slot_id = slot_id;
    step.category = category;
    step.category_name = geometry.return_zone_names.count(category) > 0 ?
      geometry.return_zone_names.at(category) : std::to_string(category);
    step.target_zone_id = category;
    step.reward = rewards[index];
    step.box_position = geometry.storage_slots.at(slot_id);
    step.startup_transition_id = leg_cost.startup_transition_id;
    step.startup_transition_position = leg_cost.startup_transition_position;
    step.pickup_transition_id = leg_cost.pickup_transition_id;
    step.pickup_transition_position = leg_cost.pickup_transition_position;
    step.pickup_stop_id = leg_cost.pickup_stop_id;
    step.pickup_stop_position = leg_cost.pickup_stop_position;
    step.return_zone_position = geometry.return_zones.at(category);
    step.step_cost = leg_cost.total;
    step.cumulative_cost = cumulative;
    step.empty_distance = leg_cost.empty_distance;
    step.loaded_distance = leg_cost.loaded_distance;
    step.turn_cost = leg_cost.turn_cost;
    step.fixed_cost = leg_cost.fixed_cost;
    result.steps.push_back(step);
    previous_index = index;
  }

  return result;
}

}  // namespace navigation::optim
