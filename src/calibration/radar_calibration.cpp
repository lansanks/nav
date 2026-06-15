#include "calibration/radar_calibration.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "maps/point_store.hpp"

namespace navigation::calibration
{
namespace
{

void setError(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

std::filesystem::path defaultConfigDir()
{
  try {
    return std::filesystem::path(ament_index_cpp::get_package_share_directory("navigation")) / "config";
  } catch (const std::exception &) {
    return std::filesystem::path("config");
  }
}

std::string sanitizedStem(const std::string & path_text)
{
  const auto stem = std::filesystem::path(path_text).stem().string();
  std::string out;
  out.reserve(stem.size());
  for (const char ch : stem) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')) {
      out.push_back(ch);
    } else if (!out.empty() && out.back() != '_') {
      out.push_back('_');
    }
  }
  while (!out.empty() && out.back() == '_') {
    out.pop_back();
  }
  return out.empty() ? "points" : out;
}

}  // namespace

std::string defaultCalibrationDataDir()
{
  return (defaultConfigDir() / "calibration" / "datas").string();
}

std::string defaultCalibrationParamsDir()
{
  return (defaultConfigDir() / "calibration" / "cali_params").string();
}

std::string resolveCalibrationDataFilePath(const std::string & path_or_name)
{
  std::filesystem::path path(path_or_name.empty() ? "radar_points" : path_or_name);
  if (!path.has_extension()) {
    path += ".yaml";
  }

  if (path.is_absolute() || path.has_parent_path()) {
    return path.string();
  }

  return (std::filesystem::path(defaultCalibrationDataDir()) / path).string();
}

std::string resolveCalibrationParamsFilePath(
  const std::string & radar_points_file,
  const std::string & mujoco_points_file)
{
  const auto name = sanitizedStem(radar_points_file) + "__to__" + sanitizedStem(mujoco_points_file) + ".yaml";
  return (std::filesystem::path(defaultCalibrationParamsDir()) / name).string();
}

std::vector<std::string> listCalibrationDataFiles()
{
  std::vector<std::string> files;
  const std::filesystem::path dir(defaultCalibrationDataDir());
  std::error_code error;
  if (!std::filesystem::exists(dir, error)) {
    return files;
  }

  for (const auto & entry : std::filesystem::directory_iterator(dir, error)) {
    if (error) {
      break;
    }
    if (!entry.is_regular_file(error)) {
      continue;
    }
    const auto extension = entry.path().extension().string();
    if (extension == ".yaml" || extension == ".yml") {
      files.push_back(entry.path().lexically_normal().string());
    }
  }

  std::sort(
    files.begin(),
    files.end(),
    [](const std::string & left, const std::string & right) {
      const auto left_name = std::filesystem::path(left).filename().string();
      const auto right_name = std::filesystem::path(right).filename().string();
      if (left_name == right_name) {
        return left < right;
      }
      return left_name < right_name;
    });
  return files;
}

bool appendRadarPoint(
  const std::string & path_or_name,
  const navigation::maps::MapPoint & point,
  std::string * saved_path,
  std::string * error_message)
{
  const auto path = resolveCalibrationDataFilePath(path_or_name);
  auto points = navigation::maps::loadPointsFile(path);
  auto next = point;
  next.id = static_cast<int>(points.size() + 1);
  next.fast = false;
  points.push_back(next);

  if (!navigation::maps::savePointsFile(path, points, error_message)) {
    return false;
  }

  if (saved_path != nullptr) {
    *saved_path = path;
  }
  return true;
}

bool saveCalibrationParams(
  const std::string & path_text,
  const std::string & radar_points_file,
  const std::string & mujoco_points_file,
  const KabschResult & result,
  std::string * error_message)
{
  const std::filesystem::path path(path_text);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      setError(error_message, "failed to create calibration directory '" + parent.string() + "': " + error.message());
      return false;
    }
  }

  std::ofstream output(path_text, std::ios::trunc);
  if (!output.is_open()) {
    setError(error_message, "failed to open calibration file for writing: " + path_text);
    return false;
  }

  output << std::fixed << std::setprecision(9);
  output << "radar_points_file: " << radar_points_file << "\n";
  output << "mujoco_points_file: " << mujoco_points_file << "\n";
  output << "point_count: " << result.errors.size() << "\n";
  output << "rotation:\n";
  output << "  - [" << result.r00 << ", " << result.r01 << "]\n";
  output << "  - [" << result.r10 << ", " << result.r11 << "]\n";
  output << "translation:\n";
  output << "  x: " << result.tx << "\n";
  output << "  y: " << result.ty << "\n";
  output << "yaw_offset: " << result.yaw_offset << "\n";
  output << "mean_error: " << result.mean_error << "\n";
  output << "max_error: " << result.max_error << "\n";
  output << "unstable: " << (result.unstable ? "true" : "false") << "\n";
  output << "errors:\n";
  for (std::size_t i = 0; i < result.errors.size(); ++i) {
    output << "  - id: " << (i + 1) << "\n";
    output << "    error: " << result.errors[i] << "\n";
  }
  return true;
}

}  // namespace navigation::calibration
