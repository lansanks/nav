#include "controller.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace navigation
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kSequentialWaypointName = "Sequential Waypoint";
constexpr const char * kEndStartupExclusionPrefix = "@end_";

double wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::msg::Twist zeroTwist()
{
  return geometry_msgs::msg::Twist{};
}

std::string lowerCopy(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

bool parseEndStartupExclusionCount(const std::string & event_label, std::size_t & count)
{
  const auto normalized = lowerCopy(event_label);
  const std::string prefix{kEndStartupExclusionPrefix};
  if (normalized.rfind(prefix, 0) != 0 || normalized.size() <= prefix.size()) {
    return false;
  }

  std::size_t parsed = 0;
  const auto * begin = normalized.data() + prefix.size();
  const auto * end = normalized.data() + normalized.size();
  const auto result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc{} || result.ptr != end) {
    return false;
  }

  count = parsed;
  return true;
}

class SequentialWaypointController final : public NavigationController
{
public:
  explicit SequentialWaypointController(const ControllerConfig & config)
  : config_(config)
  {
  }

  std::string name() const override
  {
    return kSequentialWaypointName;
  }

  void configure(const ControllerConfig & config) override
  {
    config_ = config;
  }

  void setWaypoints(const std::vector<maps::MapPoint> & points) override
  {
    waypoints_ = points;
    if (target_index_ >= waypoints_.size()) {
      target_index_ = 0;
    }
    complete_ = false;
    updateMessage();
  }

  bool start(
    std::string * error_message,
    const RobotNavigationState * initial_state = nullptr) override
  {
    if (waypoints_.empty()) {
      if (error_message != nullptr) {
        *error_message = "No waypoints loaded";
      }
      active_ = false;
      complete_ = false;
      target_index_ = 0;
      updateMessage();
      return false;
    }

    active_ = true;
    complete_ = false;
    target_index_ = startIndexForState(initial_state);
    updateMessage();
    return true;
  }

  void stop() override
  {
    active_ = false;
    updateMessage();
  }

  geometry_msgs::msg::Twist update(const RobotNavigationState & state) override
  {
    if (!active_ || waypoints_.empty() || !state.valid) {
      return zeroTwist();
    }

    advanceArrivedWaypoints(state);
    if (!active_) {
      return zeroTwist();
    }

    const auto & target = waypoints_[target_index_];
    if (isFixedSpeedSegment(target_index_)) {
      const double dx = target.x - state.x;
      const double dy = target.y - state.y;
      const double gamma = std::atan2(dy, dx);
      const double theta_g = headingBetween(waypoints_[target_index_ - 1], target);
      const double alpha = wrapAngle(gamma - state.yaw);
      const double beta = wrapAngle(theta_g - gamma);
      const bool custom_speed_segment = isCustomSpeedSegment(target_index_);
      const double linear_x = custom_speed_segment ?
        positiveOrDefault(target.segment_linear_x, config_.constant_speed_linear_x) :
        config_.constant_speed_linear_x;
      const double max_angular = custom_speed_segment ?
        positiveOrDefault(target.segment_max_angular_speed, config_.max_angular_speed) :
        config_.max_angular_speed;
      const double k_alpha = custom_speed_segment ?
        nonZeroOrDefault(target.segment_k_alpha, config_.k_alpha) :
        config_.k_alpha;
      const double k_beta = custom_speed_segment ?
        nonZeroOrDefault(target.segment_k_beta, config_.k_beta) :
        config_.k_beta;

      geometry_msgs::msg::Twist command;
      command.linear.x = linear_x;
      command.angular.z = std::clamp(
        k_alpha * alpha + k_beta * beta,
        -max_angular,
        max_angular);
      updateMessage();
      return command;
    }

    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double rho = std::hypot(dx, dy);
    const double gamma = std::atan2(dy, dx);
    const double theta_g = targetHeading(target_index_);
    const double alpha = wrapAngle(gamma - state.yaw);
    const double beta = wrapAngle(theta_g - gamma);
    const bool custom_p_segment = isCustomPControlSegment(target_index_);
    const bool fast_segment = !custom_p_segment && isFastSegment(target_index_);
    const double max_linear = custom_p_segment ?
      positiveOrDefault(target.segment_linear_x, config_.max_linear_speed) :
      (fast_segment ? config_.fast_max_linear_speed : config_.max_linear_speed);
    const double max_angular = custom_p_segment ?
      positiveOrDefault(target.segment_max_angular_speed, config_.max_angular_speed) :
      (fast_segment ? config_.fast_max_angular_speed : config_.max_angular_speed);
    const double k_rho = fast_segment ? config_.fast_k_rho : config_.k_rho;
    const double k_alpha = custom_p_segment ?
      nonZeroOrDefault(target.segment_k_alpha, config_.k_alpha) :
      (fast_segment ? config_.fast_k_alpha : config_.k_alpha);
    const double k_beta = custom_p_segment ?
      nonZeroOrDefault(target.segment_k_beta, config_.k_beta) :
      (fast_segment ? config_.fast_k_beta : config_.k_beta);

    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(k_rho * rho, 0.0, max_linear);
    command.angular.z = std::clamp(
      k_alpha * alpha + k_beta * beta,
      -max_angular,
      max_angular);
    updateMessage();
    return command;
  }

  ControllerStatus status() const override
  {
    ControllerStatus status;
    status.active = active_;
    status.complete = complete_;
    status.target_index = target_index_;
    status.point_count = waypoints_.size();
    status.message = message_;
    return status;
  }

private:
  void advanceArrivedWaypoints(const RobotNavigationState & state)
  {
    while (target_index_ < waypoints_.size()) {
      if (!hasArrivedAtTarget(state, target_index_)) {
        break;
      }

      ++target_index_;
      if (target_index_ >= waypoints_.size()) {
        active_ = false;
        complete_ = true;
        updateMessage();
        return;
      }
    }
  }

  std::size_t startIndexForState(const RobotNavigationState * state) const
  {
    if (state == nullptr || !state->valid) {
      return 0;
    }

    std::size_t best_index = 0;
    double best_distance = config_.waypoint_tolerance;
    bool found_nearby = false;
    const auto excluded_start_candidates = endStartupExcludedCandidates();
    const std::size_t candidate_count =
      waypoints_.size() > 1 ? waypoints_.size() - 1 : waypoints_.size();
    for (std::size_t i = 0; i < candidate_count; ++i) {
      if (i < excluded_start_candidates.size() && excluded_start_candidates[i]) {
        continue;
      }

      const auto & point = waypoints_[i];
      const double distance = std::hypot(point.x - state->x, point.y - state->y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
        found_nearby = true;
      }
    }

    return found_nearby ? best_index : 0;
  }

  std::vector<bool> endStartupExcludedCandidates() const
  {
    std::vector<bool> excluded(waypoints_.size(), false);
    for (std::size_t end_index = 0; end_index < waypoints_.size(); ++end_index) {
      std::size_t previous_count = 0;
      if (!parseEndStartupExclusionCount(waypoints_[end_index].event_label, previous_count) ||
        previous_count == 0)
      {
        continue;
      }

      const auto first_excluded =
        previous_count < end_index ? end_index - previous_count : std::size_t{0};
      for (std::size_t i = first_excluded; i < end_index; ++i) {
        excluded[i] = true;
      }
    }
    return excluded;
  }

  double targetHeading(std::size_t index) const
  {
    if (waypoints_.size() < 2) {
      return 0.0;
    }

    if (index + 1 < waypoints_.size()) {
      return headingBetween(waypoints_[index], waypoints_[index + 1]);
    }

    return headingBetween(waypoints_[index - 1], waypoints_[index]);
  }

  static double headingBetween(const maps::MapPoint & from, const maps::MapPoint & to)
  {
    return std::atan2(to.y - from.y, to.x - from.x);
  }

  bool isFastSegment(std::size_t target_index) const
  {
    if (target_index == 0 || target_index >= waypoints_.size()) {
      return false;
    }
    return isFastMarker(waypoints_[target_index - 1]) && isFastMarker(waypoints_[target_index]);
  }

  bool isConstantSpeedSegment(std::size_t target_index) const
  {
    if (target_index == 0 || target_index >= waypoints_.size()) {
      return false;
    }
    return isConstantSpeedMarker(waypoints_[target_index - 1]) &&
      isConstantSpeedMarker(waypoints_[target_index]);
  }

  bool isCustomSpeedSegment(std::size_t target_index) const
  {
    if (target_index == 0 || target_index >= waypoints_.size()) {
      return false;
    }
    return waypoints_[target_index].segment_custom_speed &&
      waypoints_[target_index].task_type == maps::kTaskTypeNone;
  }

  bool isFixedSpeedSegment(std::size_t target_index) const
  {
    return isConstantSpeedSegment(target_index) || isCustomConstantSpeedSegment(target_index);
  }

  bool hasArrivedAtTarget(const RobotNavigationState & state, std::size_t target_index) const
  {
    const auto & target = waypoints_[target_index];
    if (!isFixedSpeedSegment(target_index)) {
      const double rho = std::hypot(target.x - state.x, target.y - state.y);
      return rho < config_.waypoint_tolerance;
    }

    const auto & start = waypoints_[target_index - 1];
    const double segment_x = target.x - start.x;
    const double segment_y = target.y - start.y;
    const double segment_length = std::hypot(segment_x, segment_y);
    if (segment_length < 1e-6) {
      return std::hypot(target.x - state.x, target.y - state.y) < config_.waypoint_tolerance;
    }

    const double unit_x = segment_x / segment_length;
    const double unit_y = segment_y / segment_length;
    const double past_target_distance =
      (state.x - target.x) * unit_x + (state.y - target.y) * unit_y;
    return past_target_distance >= 0.0 && past_target_distance < config_.waypoint_tolerance;
  }

  static bool isFastMarker(const maps::MapPoint & point)
  {
    return point.fast && !point.constant_speed && point.task_type == maps::kTaskTypeNone;
  }

  static bool isConstantSpeedMarker(const maps::MapPoint & point)
  {
    return point.constant_speed && !point.fast && point.task_type == maps::kTaskTypeNone;
  }

  bool isCustomConstantSpeedSegment(std::size_t target_index) const
  {
    if (target_index == 0 || target_index >= waypoints_.size()) {
      return false;
    }
    return isCustomSpeedSegment(target_index) && waypoints_[target_index].segment_constant_speed;
  }

  bool isCustomPControlSegment(std::size_t target_index) const
  {
    if (target_index == 0 || target_index >= waypoints_.size()) {
      return false;
    }
    return isCustomSpeedSegment(target_index) && !waypoints_[target_index].segment_constant_speed;
  }

  static double positiveOrDefault(double value, double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  static double nonZeroOrDefault(double value, double fallback)
  {
    return std::abs(value) > 1e-9 ? value : fallback;
  }

  void updateMessage()
  {
    if (waypoints_.empty()) {
      message_ = "No waypoints";
      return;
    }

    if (complete_) {
      message_ = "Route complete";
      return;
    }

    if (active_) {
      message_ = "Target " + std::to_string(target_index_ + 1) + "/" +
        std::to_string(waypoints_.size());
      return;
    }

    message_ = "Ready: " + std::to_string(waypoints_.size()) + " points";
  }

  ControllerConfig config_;
  std::vector<maps::MapPoint> waypoints_;
  bool active_{false};
  bool complete_{false};
  std::size_t target_index_{0};
  std::string message_{"No waypoints"};
};

}  // namespace

std::vector<std::string> controllerNames()
{
  return {kSequentialWaypointName};
}

std::unique_ptr<NavigationController> createController(
  const std::string & name,
  const ControllerConfig & config)
{
  if (name == kSequentialWaypointName) {
    return std::make_unique<SequentialWaypointController>(config);
  }
  return nullptr;
}

}  // namespace navigation
