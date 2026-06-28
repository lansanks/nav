#include "interface.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "calibration/radar_calibration.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace navigation
{
namespace
{

struct RadarCalibrationTransform
{
  std::string path;
  bool enabled{false};
  double r00{1.0};
  double r01{0.0};
  double r10{0.0};
  double r11{1.0};
  double tx{0.0};
  double ty{0.0};
  double yaw_offset{0.0};
  double yaw_measurement_offset{0.0};
};

std::string trim(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string stripInlineComment(const std::string & text)
{
  const auto comment = text.find('#');
  return trim(comment == std::string::npos ? text : text.substr(0, comment));
}

bool parseScalar(const std::string & text, double & value)
{
  const auto sep = text.find(':');
  if (sep == std::string::npos) {
    return false;
  }

  try {
    value = std::stod(trim(text.substr(sep + 1)));
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseRotationRow(const std::string & text, std::array<double, 2> & row)
{
  const auto begin = text.find('[');
  const auto end = text.find(']', begin == std::string::npos ? 0 : begin);
  if (begin == std::string::npos || end == std::string::npos || end <= begin) {
    return false;
  }

  std::stringstream values(text.substr(begin + 1, end - begin - 1));
  std::string value_text;
  for (std::size_t i = 0; i < row.size(); ++i) {
    if (!std::getline(values, value_text, ',')) {
      return false;
    }
    try {
      row[i] = std::stod(trim(value_text));
    } catch (const std::exception &) {
      return false;
    }
  }
  return true;
}

std::string defaultRadarCalibrationFilePath()
{
  return "package://navigation/config/calibration/cali_params/radar_points__to__cali505.yaml";
}

std::string resolvePackageUrl(const std::string & path)
{
  const std::string prefix = "package://";
  if (path.rfind(prefix, 0) != 0) {
    return path;
  }

  const auto package_start = prefix.size();
  const auto slash = path.find('/', package_start);
  if (slash == std::string::npos) {
    return path;
  }

  const auto package_name = path.substr(package_start, slash - package_start);
  const auto relative_path = path.substr(slash + 1);
  try {
    return (std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name)) /
           relative_path).string();
  } catch (const std::exception &) {
    return path;
  }
}

bool loadRadarCalibration(
  const std::string & path,
  RadarCalibrationTransform & calibration,
  std::string & error_message)
{
  const auto resolved_path = resolvePackageUrl(path);
  std::ifstream input(resolved_path);
  if (!input.is_open()) {
    error_message = "calibration file not found: " + path;
    return false;
  }

  enum class Section { None, Rotation, Translation };
  Section section = Section::None;
  std::array<double, 2> row{};
  int rotation_row = 0;
  bool has_tx = false;
  bool has_ty = false;
  bool has_yaw_offset = false;
  std::string line;

  while (std::getline(input, line)) {
    const auto text = stripInlineComment(line);
    if (text.empty()) {
      continue;
    }

    if (text == "rotation:") {
      section = Section::Rotation;
      continue;
    }
    if (text == "translation:") {
      section = Section::Translation;
      continue;
    }
    if (text.back() == ':' && text != "rotation:" && text != "translation:") {
      section = Section::None;
      continue;
    }

    if (section == Section::Rotation && text.rfind("-", 0) == 0) {
      if (rotation_row < 2 && parseRotationRow(text, row)) {
        if (rotation_row == 0) {
          calibration.r00 = row[0];
          calibration.r01 = row[1];
        } else {
          calibration.r10 = row[0];
          calibration.r11 = row[1];
        }
        ++rotation_row;
      }
      continue;
    }

    const auto sep = text.find(':');
    if (sep == std::string::npos) {
      continue;
    }
    const auto key = trim(text.substr(0, sep));
    double value = 0.0;
    if (!parseScalar(text, value)) {
      continue;
    }

    if (section == Section::Translation && key == "x") {
      calibration.tx = value;
      has_tx = true;
    } else if (section == Section::Translation && key == "y") {
      calibration.ty = value;
      has_ty = true;
    } else if (key == "yaw_offset") {
      calibration.yaw_offset = value;
      has_yaw_offset = true;
    } else if (key == "yaw_measurement_offset") {
      calibration.yaw_measurement_offset = value;
    }
  }

  if (rotation_row != 2 || !has_tx || !has_ty || !has_yaw_offset) {
    error_message = "calibration file is missing rotation, translation, or yaw_offset: " + path;
    return false;
  }

  calibration.path = path;
  calibration.enabled = true;
  return true;
}

double yawFromQuaternionWxyz(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double normalizeYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

class RadarNavigationInterface final : public NavigationInterface
{
public:
  void start(rclcpp::Node & node) override
  {
    const auto topic = node.declare_parameter<std::string>("radar_odom_topic", "/Odometry");
    const auto calibration_file =
      node.declare_parameter<std::string>("radar_calibration_file", defaultRadarCalibrationFilePath());

    if (!calibration_file.empty()) {
      std::string error_message;
      if (setRadarCalibrationFile(calibration_file, &error_message)) {
        RCLCPP_INFO(
          node.get_logger(),
          "Radar calibration loaded from '%s'.",
          calibrationPath().c_str());
      } else {
        RCLCPP_WARN(
          node.get_logger(),
          "Radar calibration disabled: %s",
          error_message.c_str());
      }
    } else {
      RCLCPP_INFO(node.get_logger(), "Radar calibration disabled: radar_calibration_file is empty.");
    }

    // Placeholder for the real-machine radar localization path. Keep the base
    // API stable; replace only this callback when the final radar message type is fixed.
    subscription_ = node.create_subscription<nav_msgs::msg::Odometry>(
      topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        RobotNavigationState next;
        next.valid = true;
        next.stamp = rclcpp::Time(msg->header.stamp);
        next.frame_id = msg->header.frame_id;
        next.source = sourceName();

        const double raw_x = msg->pose.pose.position.x;
        const double raw_y = msg->pose.pose.position.y;
        const auto calibration = calibrationSnapshot();
        if (calibration.enabled) {
          next.x = calibration.r00 * raw_x + calibration.r01 * raw_y + calibration.tx;
          next.y = calibration.r10 * raw_x + calibration.r11 * raw_y + calibration.ty;
        } else {
          next.x = raw_x;
          next.y = raw_y;
        }
        next.z = msg->pose.pose.position.z;
        const double raw_yaw = yawFromQuaternionWxyz(
          msg->pose.pose.orientation.w,
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z);
        next.yaw = normalizeYaw(
          raw_yaw +
          (calibration.enabled ? calibration.yaw_measurement_offset + calibration.yaw_offset : 0.0));

        const double raw_linear_x = msg->twist.twist.linear.x;
        const double raw_linear_y = msg->twist.twist.linear.y;
        if (calibration.enabled) {
          next.linear_x = calibration.r00 * raw_linear_x + calibration.r01 * raw_linear_y;
          next.linear_y = calibration.r10 * raw_linear_x + calibration.r11 * raw_linear_y;
        } else {
          next.linear_x = raw_linear_x;
          next.linear_y = raw_linear_y;
        }
        next.linear_z = msg->twist.twist.linear.z;
        next.angular_z = msg->twist.twist.angular.z;
        next.planar_speed = std::hypot(next.linear_x, next.linear_y);

        std::lock_guard<std::mutex> lock(mutex_);
        state_ = next;
      });

    RCLCPP_INFO(
      node.get_logger(),
      "Radar navigation interface placeholder subscribed to '%s' as nav_msgs/Odometry.",
      topic.c_str());
  }

  bool getState(RobotNavigationState & state) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.valid) {
      return false;
    }
    state = state_;
    return true;
  }

  std::string sourceName() const override
  {
    return "radar";
  }

  bool setRadarCalibrationFile(const std::string & path, std::string * message) override
  {
    if (path.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      calibration_ = RadarCalibrationTransform{};
      if (message != nullptr) {
        *message = "radar calibration disabled";
      }
      return true;
    }

    RadarCalibrationTransform next_calibration;
    std::string error_message;
    if (!loadRadarCalibration(path, next_calibration, error_message)) {
      if (message != nullptr) {
        *message = error_message;
      }
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      calibration_ = next_calibration;
    }

    if (message != nullptr) {
      *message = "radar calibration loaded from '" + next_calibration.path + "'";
    }
    return true;
  }

private:
  RadarCalibrationTransform calibrationSnapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibration_;
  }

  std::string calibrationPath() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calibration_.path;
  }

  mutable std::mutex mutex_;
  RobotNavigationState state_;
  RadarCalibrationTransform calibration_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace

std::unique_ptr<NavigationInterface> createRadarInterface()
{
  return std::make_unique<RadarNavigationInterface>();
}

}  // namespace navigation
