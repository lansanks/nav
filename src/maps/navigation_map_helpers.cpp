#include "maps/navigation_map_helpers.hpp"

#include <cstddef>
#include <filesystem>
#include <system_error>

#include "maps/point_store.hpp"

namespace navigation::maps
{
namespace
{

std::string normalizedPathText(const std::string & path)
{
  return std::filesystem::path(path).lexically_normal().string();
}

}  // namespace

int findPathIndex(const std::vector<std::string> & paths, const std::string & path)
{
  const auto target = normalizedPathText(path);
  for (std::size_t i = 0; i < paths.size(); ++i) {
    if (normalizedPathText(paths[i]) == target) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void addExistingPath(std::vector<std::string> & paths, const std::string & path)
{
  if (path.empty() || findPathIndex(paths, path) >= 0) {
    return;
  }

  std::error_code error;
  const std::filesystem::path candidate(path);
  if (std::filesystem::exists(candidate, error) && std::filesystem::is_regular_file(candidate, error)) {
    paths.push_back(candidate.lexically_normal().string());
  }
}

std::vector<std::string> makePathLabels(const std::vector<std::string> & paths)
{
  std::vector<std::string> labels;
  labels.reserve(paths.size());

  for (std::size_t i = 0; i < paths.size(); ++i) {
    const std::filesystem::path path(paths[i]);
    auto label = path.filename().string();
    bool duplicate_name = false;
    for (std::size_t j = 0; j < paths.size(); ++j) {
      if (i != j && std::filesystem::path(paths[j]).filename() == path.filename()) {
        duplicate_name = true;
        break;
      }
    }

    if (duplicate_name && !path.parent_path().filename().empty()) {
      label = path.parent_path().filename().string() + "/" + label;
    }
    labels.push_back(label);
  }

  return labels;
}

std::string defaultNewPointsName()
{
  const auto default_dir = std::filesystem::path(defaultPointsFilePath()).parent_path();
  for (int index = 1; index < 1000; ++index) {
    const auto name = index == 1 ?
      std::string("new_points.yaml") :
      "new_points_" + std::to_string(index) + ".yaml";
    std::error_code error;
    if (!std::filesystem::exists(default_dir / name, error)) {
      return name;
    }
  }

  return "new_points.yaml";
}

bool validateFastMarkers(const std::vector<MapPoint> & points, std::string * error_message)
{
  for (std::size_t i = 0; i < points.size();) {
    if (!points[i].fast) {
      ++i;
      continue;
    }

    if (i + 1 < points.size() && points[i + 1].fast) {
      i += 2;
      continue;
    }

    if (error_message != nullptr) {
      *error_message = "Fast red points must be adjacent pairs";
    }
    return false;
  }
  return true;
}

int pendingFastMarkerIndex(const std::vector<MapPoint> & points)
{
  int pending_index = -1;
  for (std::size_t i = 0; i < points.size();) {
    if (!points[i].fast) {
      ++i;
      continue;
    }

    if (i + 1 < points.size() && points[i + 1].fast) {
      i += 2;
      continue;
    }

    if (pending_index >= 0) {
      return -2;
    }
    pending_index = static_cast<int>(i);
    ++i;
  }
  return pending_index;
}

}  // namespace navigation::maps
