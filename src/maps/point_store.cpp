#include "maps/point_store.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace navigation::maps
{
namespace
{

std::string trim(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

void setError(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

bool parseBool(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value == "true" || value == "1" || value == "yes" || value == "on";
}

std::uint8_t parseTaskType(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (value == "pickup" || value == "1") {
    return kTaskTypePickup;
  }
  if (value == "place" || value == "placed" || value == "2") {
    return kTaskTypePlace;
  }
  return kTaskTypeNone;
}

const char * taskTypeText(std::uint8_t task_type)
{
  if (task_type == kTaskTypePickup) {
    return "pickup";
  }
  if (task_type == kTaskTypePlace) {
    return "place";
  }
  return "none";
}

std::string unquoteScalar(std::string value)
{
  value = trim(value);
  if (value.size() < 2 || value.front() != '"' || value.back() != '"') {
    return value;
  }

  std::string output;
  output.reserve(value.size() - 2);
  bool escaping = false;
  for (std::size_t i = 1; i + 1 < value.size(); ++i) {
    const char ch = value[i];
    if (escaping) {
      output.push_back(ch);
      escaping = false;
    } else if (ch == '\\') {
      escaping = true;
    } else {
      output.push_back(ch);
    }
  }
  return output;
}

std::string quoteScalar(const std::string & value)
{
  std::string output = "\"";
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') {
      output.push_back('\\');
    }
    output.push_back(ch);
  }
  output.push_back('"');
  return output;
}

}  // namespace

std::string defaultPointsFilePath()
{
  try {
    return (
      std::filesystem::path(ament_index_cpp::get_package_share_directory("navigation")) /
      "config" / "points" / "points.yaml").string();
  } catch (const std::exception &) {
    return (std::filesystem::path("config") / "points" / "points.yaml").string();
  }
}

std::string resolvePointsFilePath(const std::string & path_or_name)
{
  if (path_or_name.empty()) {
    return defaultPointsFilePath();
  }

  std::filesystem::path path(path_or_name);
  if (!path.has_extension()) {
    path += ".yaml";
  }

  if (path.is_absolute() || path.has_parent_path()) {
    return path.string();
  }

  return (std::filesystem::path(defaultPointsFilePath()).parent_path() / path).string();
}

std::vector<std::string> listPointsFiles(const std::string & include_path)
{
  std::vector<std::string> files;

  auto add_file = [&](const std::filesystem::path & path) {
    const auto extension = path.extension().string();
    if (extension != ".yaml" && extension != ".yml") {
      return;
    }

    const auto normalized = path.lexically_normal().string();
    if (std::find(files.begin(), files.end(), normalized) == files.end()) {
      files.push_back(normalized);
    }
  };

  const auto default_dir = std::filesystem::path(defaultPointsFilePath()).parent_path();
  std::error_code error;
  if (!default_dir.empty() && std::filesystem::exists(default_dir, error)) {
    for (const auto & entry : std::filesystem::directory_iterator(default_dir, error)) {
      if (error) {
        break;
      }
      if (entry.is_regular_file(error)) {
        add_file(entry.path());
      }
    }
  }

  if (!include_path.empty()) {
    const std::filesystem::path current(include_path);
    if (std::filesystem::exists(current, error) && std::filesystem::is_regular_file(current, error)) {
      add_file(current);
    }
  }

  std::sort(
    files.begin(),
    files.end(),
    [](const std::string & left, const std::string & right) {
      const std::filesystem::path left_path(left);
      const std::filesystem::path right_path(right);
      const auto left_name = left_path.filename().string();
      const auto right_name = right_path.filename().string();
      if (left_name == right_name) {
        return left < right;
      }
      return left_name < right_name;
    });

  return files;
}

std::vector<MapPoint> loadPointsFile(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    return {};
  }

  std::vector<MapPoint> points;
  MapPoint current;
  bool in_point = false;
  std::string line;

  auto commit_point = [&]() {
    if (!in_point) {
      return;
    }
    points.push_back(current);
    current = MapPoint{};
    in_point = false;
  };

  while (std::getline(input, line)) {
    auto text = trim(line);
    if (text.empty() || text[0] == '#') {
      continue;
    }

    if (text.rfind("-", 0) == 0) {
      commit_point();
      in_point = true;
      text = trim(text.substr(1));
    }

    const auto sep = text.find(':');
    if (sep == std::string::npos) {
      continue;
    }

    const auto key = trim(text.substr(0, sep));
    const auto value = trim(text.substr(sep + 1));
    if (!in_point && key != "points") {
      continue;
    }

    try {
      if (key == "id") {
        current.id = std::stoi(value);
      } else if (key == "x") {
        current.x = std::stod(value);
      } else if (key == "y") {
        current.y = std::stod(value);
      } else if (key == "fast") {
        current.fast = parseBool(value);
      } else if (key == "task_type") {
        current.task_type = parseTaskType(value);
      } else if (key == "event_label") {
        current.event_label = unquoteScalar(value);
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  commit_point();

  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].id = static_cast<int>(i + 1);
  }
  return points;
}

bool savePointsFile(const std::string & path_text, const std::vector<MapPoint> & points, std::string * error_message)
{
  const std::filesystem::path path(path_text);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      setError(error_message, "failed to create directory '" + parent.string() + "': " + error.message());
      return false;
    }
  }

  std::ofstream output(path_text, std::ios::trunc);
  if (!output.is_open()) {
    setError(error_message, "failed to open file for writing: " + path_text);
    return false;
  }

  output << std::fixed << std::setprecision(6);
  output << "points:\n";
  for (const auto & point : points) {
    output << "  - id: " << point.id << "\n";
    output << "    x: " << point.x << "\n";
    output << "    y: " << point.y << "\n";
    output << "    fast: " << (point.fast ? "true" : "false") << "\n";
    output << "    task_type: " << taskTypeText(point.task_type) << "\n";
    output << "    event_label: " << quoteScalar(point.event_label) << "\n";
  }
  return true;
}

}  // namespace navigation::maps
