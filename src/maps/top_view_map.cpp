#include "maps/top_view_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "tinyxml2.h"
#include "ui/map_ui_renderer.hpp"

namespace navigation::maps
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kImageMapPixelsPerMeter = 100.0;

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Transform2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Bounds
{
  bool valid{false};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};

  void include(Vec2 point)
  {
    if (!valid) {
      valid = true;
      min_x = max_x = point.x;
      min_y = max_y = point.y;
      return;
    }
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  void includeCircle(Vec2 center, double radius)
  {
    include({center.x - radius, center.y - radius});
    include({center.x + radius, center.y + radius});
  }

  void addMargin(double margin)
  {
    if (!valid) {
      min_x = -5.0;
      max_x = 5.0;
      min_y = -5.0;
      max_y = 5.0;
      valid = true;
      return;
    }
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
  }
};

enum class MapGeomType
{
  Box,
  Cylinder,
};

struct MeshAsset
{
  std::filesystem::path path;
  double scale_x{1.0};
  double scale_y{1.0};
};

struct MapGeom
{
  MapGeomType type{MapGeomType::Box};
  std::vector<Vec2> corners;
  Vec2 center;
  double radius{0.0};
  cv::Scalar fill{120, 120, 120};
  cv::Scalar outline{55, 55, 55};
};

std::vector<double> parseDoubles(const char * text)
{
  std::vector<double> values;
  if (text == nullptr) {
    return values;
  }

  const char * cursor = text;
  char * end = nullptr;
  while (*cursor != '\0') {
    const double value = std::strtod(cursor, &end);
    if (end == cursor) {
      ++cursor;
      continue;
    }
    values.push_back(value);
    cursor = end;
  }
  return values;
}

std::array<double, 3> readVec3(const tinyxml2::XMLElement & element, const char * attribute)
{
  std::array<double, 3> result{0.0, 0.0, 0.0};
  const auto values = parseDoubles(element.Attribute(attribute));
  for (std::size_t i = 0; i < std::min<std::size_t>(3, values.size()); ++i) {
    result[i] = values[i];
  }
  return result;
}

std::array<double, 4> readQuatWxyz(const tinyxml2::XMLElement & element)
{
  std::array<double, 4> result{1.0, 0.0, 0.0, 0.0};
  const auto values = parseDoubles(element.Attribute("quat"));
  for (std::size_t i = 0; i < std::min<std::size_t>(4, values.size()); ++i) {
    result[i] = values[i];
  }
  return result;
}

double yawFromQuatWxyz(const std::array<double, 4> & q)
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double readYaw(const tinyxml2::XMLElement & element)
{
  if (element.Attribute("quat") != nullptr) {
    return yawFromQuatWxyz(readQuatWxyz(element));
  }

  const auto euler = parseDoubles(element.Attribute("euler"));
  if (euler.size() >= 3) {
    return euler[2];
  }

  return 0.0;
}

Vec2 rotate(Vec2 point, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * point.x - s * point.y, s * point.x + c * point.y};
}

Vec2 transformPoint(const Transform2D & transform, Vec2 point)
{
  const auto rotated = rotate(point, transform.yaw);
  return {transform.x + rotated.x, transform.y + rotated.y};
}

Transform2D composeTransform(const Transform2D & parent, const Transform2D & child)
{
  const auto translated = transformPoint(parent, {child.x, child.y});
  return {translated.x, translated.y, parent.yaw + child.yaw};
}

cv::Scalar readColor(const tinyxml2::XMLElement & element, cv::Scalar fallback)
{
  const auto values = parseDoubles(element.Attribute("rgba"));
  if (values.size() < 3) {
    return fallback;
  }

  const double r = std::clamp(values[0], 0.0, 1.0) * 255.0;
  const double g = std::clamp(values[1], 0.0, 1.0) * 255.0;
  const double b = std::clamp(values[2], 0.0, 1.0) * 255.0;
  return cv::Scalar(b, g, r);
}

std::array<double, 3> readVec3OrDefault(
  const tinyxml2::XMLElement & element,
  const char * attribute,
  const std::array<double, 3> & fallback)
{
  auto result = fallback;
  const auto values = parseDoubles(element.Attribute(attribute));
  for (std::size_t i = 0; i < std::min<std::size_t>(3, values.size()); ++i) {
    result[i] = values[i];
  }
  return result;
}

std::vector<Vec2> convexHull(std::vector<Vec2> points)
{
  std::sort(
    points.begin(),
    points.end(),
    [](const Vec2 & left, const Vec2 & right) {
      if (std::abs(left.x - right.x) > 1e-9) {
        return left.x < right.x;
      }
      return left.y < right.y;
    });

  points.erase(
    std::unique(
      points.begin(),
      points.end(),
      [](const Vec2 & left, const Vec2 & right) {
        return std::abs(left.x - right.x) <= 1e-9 && std::abs(left.y - right.y) <= 1e-9;
      }),
    points.end());

  if (points.size() <= 2) {
    return points;
  }

  auto cross = [](const Vec2 & origin, const Vec2 & a, const Vec2 & b) {
      return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
    };

  std::vector<Vec2> hull;
  hull.reserve(points.size() * 2);
  for (const auto & point : points) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }

  const auto lower_size = hull.size();
  for (auto iter = points.rbegin() + 1; iter != points.rend(); ++iter) {
    while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *iter) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*iter);
  }

  if (!hull.empty()) {
    hull.pop_back();
  }
  return hull;
}

std::vector<Vec2> readObjVertices2D(const MeshAsset & mesh)
{
  std::ifstream input(mesh.path);
  if (!input.is_open()) {
    return {};
  }

  std::vector<Vec2> vertices;
  std::string line;
  while (std::getline(input, line)) {
    if (line.size() < 2 || line[0] != 'v' || line[1] != ' ') {
      continue;
    }

    const char * cursor = line.c_str() + 2;
    char * end = nullptr;
    const double x = std::strtod(cursor, &end);
    if (end == cursor) {
      continue;
    }
    cursor = end;
    const double y = std::strtod(cursor, &end);
    if (end == cursor) {
      continue;
    }
    vertices.push_back({x * mesh.scale_x, y * mesh.scale_y});
  }

  return vertices;
}

std::string sceneFileFromAlias(const std::string & scene)
{
  if (scene == "flat" || scene == "plane") {
    return "scene_flat.xml";
  }
  if (scene == "obstacle") {
    return "scene.xml";
  }
  if (scene == "terrain") {
    return "scene_terrain.xml";
  }
  if (scene == "robocon") {
    return "scene_robocon.xml";
  }
  return scene;
}

bool isImageMapFile(const std::filesystem::path & path)
{
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp";
}

std::filesystem::path navigationMapsDir()
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory("navigation")) /
         "config" / "maps";
}

std::filesystem::path robotDescriptionShareDir()
{
  return ament_index_cpp::get_package_share_directory("robot_description");
}

cv::Scalar mapBackground(bool light_theme)
{
  return light_theme ? cv::Scalar(245, 245, 245) : cv::Scalar(18, 18, 18);
}

cv::Scalar mapGrid(bool light_theme)
{
  return light_theme ? cv::Scalar(222, 222, 222) : cv::Scalar(40, 40, 40);
}

cv::Scalar mapGridAxis(bool light_theme)
{
  return light_theme ? cv::Scalar(178, 178, 178) : cv::Scalar(76, 76, 76);
}

cv::Scalar mapSurface(bool light_theme)
{
  return light_theme ? cv::Scalar(255, 255, 255) : cv::Scalar(28, 28, 28);
}

cv::Scalar mapSurfaceBorder(bool light_theme)
{
  return light_theme ? cv::Scalar(178, 178, 178) : cv::Scalar(76, 76, 76);
}

cv::Scalar mapText(bool light_theme)
{
  return light_theme ? cv::Scalar(28, 28, 28) : cv::Scalar(232, 232, 232);
}

cv::Scalar mapMutedText(bool light_theme)
{
  return light_theme ? cv::Scalar(92, 92, 92) : cv::Scalar(182, 182, 182);
}

}  // namespace

std::string resolveScenePath(const std::string & robot_name, const std::string & scene)
{
  const std::filesystem::path requested(scene);
  if (requested.is_absolute() && std::filesystem::exists(requested)) {
    return requested.string();
  }
  if (requested.has_parent_path() && std::filesystem::exists(requested)) {
    return requested.string();
  }

  const auto scene_file = sceneFileFromAlias(scene);
  try {
    const auto maps_dir = navigationMapsDir();
    const std::filesystem::path candidate = maps_dir / robot_name / scene_file;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }

    const std::filesystem::path fallback = maps_dir / robot_name / "scene_flat.xml";
    if (std::filesystem::exists(fallback)) {
      return fallback.string();
    }
  } catch (const std::exception &) {
  }

  try {
    const auto share_dir = robotDescriptionShareDir();
    const std::filesystem::path candidate = share_dir / robot_name / scene_file;
    if (std::filesystem::exists(candidate)) {
      return candidate.string();
    }

    const std::filesystem::path fallback = share_dir / robot_name / "scene_flat.xml";
    return fallback.string();
  } catch (const std::exception &) {
  }

  return (std::filesystem::path("config") / "maps" / robot_name / "scene_flat.xml").string();
}

std::vector<std::string> listSceneFiles(const std::string & robot_name)
{
  std::vector<std::string> files;
  std::error_code error;

  auto add_scene_file = [&](const std::filesystem::path & path) {
    const bool is_xml = path.extension() == ".xml";
    const bool is_image = isImageMapFile(path);
    if (!is_xml && !is_image) {
      return;
    }

    const auto filename = path.filename().string();
    if (is_image && filename.find("preview") != std::string::npos) {
      return;
    }
    if (filename.find("scene") == std::string::npos && filename.find("sence") == std::string::npos) {
      return;
    }

    const auto normalized = path.lexically_normal().string();
    if (std::find(files.begin(), files.end(), normalized) == files.end()) {
      files.push_back(normalized);
    }
  };

  auto collect_scene_files = [&](const std::filesystem::path & scene_dir) {
    error.clear();
    if (!std::filesystem::exists(scene_dir, error)) {
      return;
    }

    for (const auto & entry : std::filesystem::directory_iterator(scene_dir, error)) {
      if (error) {
        break;
      }
      if (entry.is_regular_file(error)) {
        add_scene_file(entry.path());
      }
    }
  };

  try {
    collect_scene_files(navigationMapsDir() / robot_name);
  } catch (const std::exception &) {
  }

  try {
    collect_scene_files(robotDescriptionShareDir() / robot_name);
  } catch (const std::exception &) {
  }

  std::sort(
    files.begin(),
    files.end(),
    [](const std::string & left, const std::string & right) {
      return std::filesystem::path(left).filename().string() <
             std::filesystem::path(right).filename().string();
    });

  return files;
}

struct TopViewMap::Impl
{
  Impl(int width, int height, double padding)
  : width(std::max(320, width)),
    height(std::max(240, height)),
    padding(std::max(10.0, padding)),
    ui_renderer(this->width, this->height)
  {
  }

  void load(const std::string & path)
  {
    scene_path = path;
    scene_dir = std::filesystem::path(path).parent_path();
    if (isImageMapFile(path)) {
      loadImageMap(path);
      return;
    }

    image_background.release();
    tinyxml2::XMLDocument document;
    const auto result = document.LoadFile(path.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("failed to load scene xml: " + path);
    }

    const auto * root = document.FirstChildElement("mujoco");
    if (root == nullptr) {
      throw std::runtime_error("scene xml has no <mujoco> root: " + path);
    }

    readAssets(*root);

    const auto * world = root->FirstChildElement("worldbody");
    if (world == nullptr) {
      throw std::runtime_error("scene xml has no <worldbody>: " + path);
    }

    geoms.clear();
    bounds = Bounds{};
    parseChildren(*world, Transform2D{});
    bounds.addMargin(0.8);
    computeScale();
  }

  void loadImageMap(const std::string & path)
  {
    image_background = cv::imread(path, cv::IMREAD_COLOR);
    if (image_background.empty()) {
      throw std::runtime_error("failed to load map image: " + path);
    }

    hfields.clear();
    materials.clear();
    meshes.clear();
    geoms.clear();
    bounds = Bounds{};
    bounds.include({0.0, 0.0});
    bounds.include({
      static_cast<double>(image_background.cols) / kImageMapPixelsPerMeter,
      static_cast<double>(image_background.rows) / kImageMapPixelsPerMeter,
    });
    computeScale();
  }

  cv::Mat draw(const navigation::RobotNavigationState * state, const MapUiState & ui_state) const
  {
    cv::Mat canvas(height, ui_renderer.canvasWidth(ui_state), CV_8UC3, mapBackground(ui_state.light_theme));
    if (!image_background.empty()) {
      drawImageBackground(canvas);
    } else {
      drawGrid(canvas, ui_state.light_theme);
      for (const auto & geom : geoms) {
        drawGeom(canvas, geom);
      }
    }
    drawSavedPoints(canvas, ui_state);
    drawRoutePatch(canvas, ui_state);
    drawOptimizedPlan(canvas, ui_state);

    if (state != nullptr && state->valid) {
      drawRobot(canvas, *state);
      drawStatus(canvas, *state, ui_state);
    } else {
      cv::putText(
        canvas,
        "waiting for pose",
        cv::Point(18, 78),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        mapText(ui_state.light_theme),
        2,
        cv::LINE_AA);
    }

    cv::putText(
      canvas,
      std::filesystem::path(scene_path).filename().string(),
      cv::Point(18, height - 18),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      mapMutedText(ui_state.light_theme),
      1,
      cv::LINE_AA);

    ui_renderer.draw(canvas, ui_state, points.size());
    return canvas;
  }

  MapUiHit hitTestUi(int pixel_x, int pixel_y, const MapUiState & ui_state) const
  {
    return ui_renderer.hitTest(pixel_x, pixel_y, ui_state);
  }

  void readAssets(const tinyxml2::XMLElement & root)
  {
    hfields.clear();
    materials.clear();
    meshes.clear();

    mesh_dir = scene_dir / "meshes";
    if (const auto * compiler = root.FirstChildElement("compiler"); compiler != nullptr) {
      if (const auto * meshdir = compiler->Attribute("meshdir"); meshdir != nullptr) {
        const std::filesystem::path compiler_mesh_dir(meshdir);
        mesh_dir = compiler_mesh_dir.is_absolute() ? compiler_mesh_dir : scene_dir / compiler_mesh_dir;
      }
    }

    for (const auto * asset = root.FirstChildElement("asset");
      asset != nullptr;
      asset = asset->NextSiblingElement("asset"))
    {
      for (const auto * material = asset->FirstChildElement("material");
        material != nullptr;
        material = material->NextSiblingElement("material"))
      {
        const auto * name = material->Attribute("name");
        if (name == nullptr) {
          continue;
        }
        materials[name] = readColor(*material, cv::Scalar(150, 145, 120));
      }

      for (const auto * hfield = asset->FirstChildElement("hfield");
        hfield != nullptr;
        hfield = hfield->NextSiblingElement("hfield"))
      {
        const auto * name = hfield->Attribute("name");
        if (name == nullptr) {
          continue;
        }
        const auto size = readVec3(*hfield, "size");
        hfields[name] = {std::max(0.1, size[0]), std::max(0.1, size[1])};
      }

      for (const auto * mesh = asset->FirstChildElement("mesh");
        mesh != nullptr;
        mesh = mesh->NextSiblingElement("mesh"))
      {
        const auto * name = mesh->Attribute("name");
        const auto * file = mesh->Attribute("file");
        if (name == nullptr || file == nullptr) {
          continue;
        }

        const std::filesystem::path file_path(file);
        const auto scale = readVec3OrDefault(*mesh, "scale", {1.0, 1.0, 1.0});
        meshes[name] = {
          file_path.is_absolute() ? file_path : mesh_dir / file_path,
          scale[0],
          scale[1],
        };
      }
    }
  }

  void parseChildren(const tinyxml2::XMLElement & parent, const Transform2D & transform)
  {
    for (const auto * child = parent.FirstChildElement();
      child != nullptr;
      child = child->NextSiblingElement())
    {
      const std::string tag = child->Name() == nullptr ? "" : child->Name();
      if (tag == "geom") {
        parseGeom(*child, transform);
      } else if (tag == "body") {
        const auto pos = readVec3(*child, "pos");
        const Transform2D child_transform{pos[0], pos[1], readYaw(*child)};
        parseChildren(*child, composeTransform(transform, child_transform));
      }
    }
  }

  void parseGeom(const tinyxml2::XMLElement & geom, const Transform2D & parent_transform)
  {
    const std::string type = geom.Attribute("type") == nullptr ? "sphere" : geom.Attribute("type");
    const bool is_mesh = type == "mesh" || geom.Attribute("mesh") != nullptr;
    if (type == "plane") {
      return;
    }

    const auto pos = readVec3(geom, "pos");
    const Transform2D local{pos[0], pos[1], readYaw(geom)};
    const auto transform = composeTransform(parent_transform, local);
    const auto size = readVec3(geom, "size");
    const auto fill = readGeomColor(geom, cv::Scalar(150, 145, 120));

    if (is_mesh) {
      const auto * mesh_name = geom.Attribute("mesh");
      if (mesh_name != nullptr) {
        const auto found = meshes.find(mesh_name);
        if (found != meshes.end()) {
          addMesh(transform, found->second, fill);
        }
      }
      return;
    }

    if (type == "box") {
      const double sx = std::max(0.01, size[0]);
      const double sy = std::max(0.01, size[1]);
      addBox(transform, sx, sy, fill);
      return;
    }

    if (type == "cylinder") {
      const double radius = std::max(0.02, size[0]);
      addCylinder({transform.x, transform.y}, radius, readGeomColor(geom, cv::Scalar(90, 115, 210)));
      return;
    }

    if (type == "hfield") {
      const auto * hfield_name = geom.Attribute("hfield");
      auto half_extents = std::pair<double, double>{0.5, 0.5};
      if (hfield_name != nullptr) {
        const auto found = hfields.find(hfield_name);
        if (found != hfields.end()) {
          half_extents = found->second;
        }
      }
      addBox(transform, half_extents.first, half_extents.second, cv::Scalar(75, 112, 82));
    }
  }

  cv::Scalar readGeomColor(const tinyxml2::XMLElement & geom, cv::Scalar fallback) const
  {
    const auto explicit_color = parseDoubles(geom.Attribute("rgba"));
    if (explicit_color.size() >= 3) {
      return readColor(geom, fallback);
    }

    const auto * material = geom.Attribute("material");
    if (material != nullptr) {
      const auto found = materials.find(material);
      if (found != materials.end()) {
        return found->second;
      }
    }

    return fallback;
  }

  void addMesh(const Transform2D & transform, const MeshAsset & mesh, cv::Scalar fill)
  {
    auto vertices = readObjVertices2D(mesh);
    if (vertices.size() < 3) {
      return;
    }

    for (auto & vertex : vertices) {
      vertex = transformPoint(transform, vertex);
    }

    auto hull = convexHull(std::move(vertices));
    if (hull.size() < 3) {
      return;
    }

    MapGeom geom;
    geom.type = MapGeomType::Box;
    geom.fill = fill;
    geom.outline = cv::Scalar(45, 48, 52);
    geom.corners = std::move(hull);
    for (const auto & corner : geom.corners) {
      bounds.include(corner);
    }
    geoms.push_back(std::move(geom));
  }

  void addBox(const Transform2D & transform, double sx, double sy, cv::Scalar fill)
  {
    MapGeom geom;
    geom.type = MapGeomType::Box;
    geom.fill = fill;
    geom.outline = cv::Scalar(45, 48, 52);

    const std::array<Vec2, 4> local_corners{
      Vec2{-sx, -sy},
      Vec2{sx, -sy},
      Vec2{sx, sy},
      Vec2{-sx, sy},
    };

    for (const auto & corner : local_corners) {
      const auto world_corner = transformPoint(transform, corner);
      geom.corners.push_back(world_corner);
      bounds.include(world_corner);
    }

    geoms.push_back(std::move(geom));
  }

  void addCylinder(Vec2 center, double radius, cv::Scalar fill)
  {
    MapGeom geom;
    geom.type = MapGeomType::Cylinder;
    geom.center = center;
    geom.radius = radius;
    geom.fill = fill;
    geom.outline = cv::Scalar(45, 48, 52);
    bounds.includeCircle(center, radius);
    geoms.push_back(std::move(geom));
  }

  void computeScale()
  {
    const double map_width = std::max(1e-3, bounds.max_x - bounds.min_x);
    const double map_height = std::max(1e-3, bounds.max_y - bounds.min_y);
    const double available_width = std::max(1.0, static_cast<double>(width) - 2.0 * padding);
    const double available_height = std::max(1.0, static_cast<double>(height) - 2.0 * padding);
    fit_scale = std::min(available_width / map_width, available_height / map_height);
    scale = fit_scale;
    view_left = 0.5 * (static_cast<double>(width) - map_width * scale);
    view_top = 0.5 * (static_cast<double>(height) - map_height * scale);
  }

  cv::Point worldToPixel(Vec2 point) const
  {
    const int px = static_cast<int>(std::lround(view_left + (point.x - bounds.min_x) * scale));
    const int py = static_cast<int>(std::lround(view_top + (bounds.max_y - point.y) * scale));
    return {px, py};
  }

  bool pixelToWorld(int pixel_x, int pixel_y, MapPoint & point) const
  {
    if (!bounds.valid || scale <= 1e-9) {
      return false;
    }

    const double world_x = bounds.min_x + (static_cast<double>(pixel_x) - view_left) / scale;
    const double world_y = bounds.max_y - (static_cast<double>(pixel_y) - view_top) / scale;
    if (world_x < bounds.min_x || world_x > bounds.max_x ||
      world_y < bounds.min_y || world_y > bounds.max_y)
    {
      return false;
    }

    point.x = world_x;
    point.y = world_y;
    return true;
  }

  bool zoomAt(int pixel_x, int pixel_y, double factor)
  {
    if (!bounds.valid || scale <= 1e-9 || fit_scale <= 1e-9 ||
      factor <= 0.0 || pixel_x < 0 || pixel_x >= width || pixel_y < 0 || pixel_y >= height)
    {
      return false;
    }

    const double anchor_x = bounds.min_x + (static_cast<double>(pixel_x) - view_left) / scale;
    const double anchor_y = bounds.max_y - (static_cast<double>(pixel_y) - view_top) / scale;
    const double new_scale = std::clamp(scale * factor, fit_scale * 0.25, fit_scale * 24.0);
    if (std::abs(new_scale - scale) <= 1e-9) {
      return false;
    }

    scale = new_scale;
    view_left = static_cast<double>(pixel_x) - (anchor_x - bounds.min_x) * scale;
    view_top = static_cast<double>(pixel_y) - (bounds.max_y - anchor_y) * scale;
    return true;
  }

  bool panBy(int delta_x, int delta_y)
  {
    if (!bounds.valid || scale <= 1e-9 || (delta_x == 0 && delta_y == 0)) {
      return false;
    }

    view_left += static_cast<double>(delta_x);
    view_top += static_cast<double>(delta_y);
    return true;
  }

  void drawImageBackground(cv::Mat & canvas) const
  {
    if (image_background.empty() || !bounds.valid || scale <= 1e-9) {
      return;
    }

    const double map_width = std::max(1e-3, bounds.max_x - bounds.min_x);
    const double map_height = std::max(1e-3, bounds.max_y - bounds.min_y);
    const double dst_left = view_left;
    const double dst_top = view_top;
    const double dst_width = map_width * scale;
    const double dst_height = map_height * scale;
    if (dst_width <= 1.0 || dst_height <= 1.0) {
      return;
    }

    const int clip_left = std::max(0, static_cast<int>(std::floor(dst_left)));
    const int clip_top = std::max(0, static_cast<int>(std::floor(dst_top)));
    const int clip_right = std::min(width, static_cast<int>(std::ceil(dst_left + dst_width)));
    const int clip_bottom = std::min(height, static_cast<int>(std::ceil(dst_top + dst_height)));
    if (clip_left >= clip_right || clip_top >= clip_bottom) {
      return;
    }

    const auto source_x0 = std::clamp(
      (static_cast<double>(clip_left) - dst_left) / dst_width * image_background.cols,
      0.0,
      static_cast<double>(image_background.cols));
    const auto source_y0 = std::clamp(
      (static_cast<double>(clip_top) - dst_top) / dst_height * image_background.rows,
      0.0,
      static_cast<double>(image_background.rows));
    const auto source_x1 = std::clamp(
      (static_cast<double>(clip_right) - dst_left) / dst_width * image_background.cols,
      0.0,
      static_cast<double>(image_background.cols));
    const auto source_y1 = std::clamp(
      (static_cast<double>(clip_bottom) - dst_top) / dst_height * image_background.rows,
      0.0,
      static_cast<double>(image_background.rows));

    const int src_x = std::clamp(
      static_cast<int>(std::floor(source_x0)),
      0,
      std::max(0, image_background.cols - 1));
    const int src_y = std::clamp(
      static_cast<int>(std::floor(source_y0)),
      0,
      std::max(0, image_background.rows - 1));
    const int src_right = std::clamp(
      static_cast<int>(std::ceil(source_x1)),
      src_x + 1,
      image_background.cols);
    const int src_bottom = std::clamp(
      static_cast<int>(std::ceil(source_y1)),
      src_y + 1,
      image_background.rows);

    const cv::Rect src_rect(src_x, src_y, src_right - src_x, src_bottom - src_y);
    const cv::Rect dst_rect(clip_left, clip_top, clip_right - clip_left, clip_bottom - clip_top);
    cv::Mat resized;
    const int interpolation =
      dst_rect.width < src_rect.width || dst_rect.height < src_rect.height ? cv::INTER_AREA : cv::INTER_LINEAR;
    cv::resize(image_background(src_rect), resized, dst_rect.size(), 0.0, 0.0, interpolation);
    resized.copyTo(canvas(dst_rect));

    const cv::Point border_top_left(
      static_cast<int>(std::lround(dst_left)),
      static_cast<int>(std::lround(dst_top)));
    const cv::Point border_bottom_right(
      static_cast<int>(std::lround(dst_left + dst_width)),
      static_cast<int>(std::lround(dst_top + dst_height)));
    cv::rectangle(canvas, border_top_left, border_bottom_right, cv::Scalar(25, 25, 25), 2, cv::LINE_AA);
  }

  void drawGrid(cv::Mat & canvas, bool light_theme) const
  {
    const int start_x = static_cast<int>(std::floor(bounds.min_x));
    const int end_x = static_cast<int>(std::ceil(bounds.max_x));
    const int start_y = static_cast<int>(std::floor(bounds.min_y));
    const int end_y = static_cast<int>(std::ceil(bounds.max_y));

    for (int x = start_x; x <= end_x; ++x) {
      const auto p0 = worldToPixel({static_cast<double>(x), bounds.min_y});
      const auto p1 = worldToPixel({static_cast<double>(x), bounds.max_y});
      const auto color = x == 0 ? mapGridAxis(light_theme) : mapGrid(light_theme);
      cv::line(canvas, p0, p1, color, x == 0 ? 2 : 1, cv::LINE_AA);
    }

    for (int y = start_y; y <= end_y; ++y) {
      const auto p0 = worldToPixel({bounds.min_x, static_cast<double>(y)});
      const auto p1 = worldToPixel({bounds.max_x, static_cast<double>(y)});
      const auto color = y == 0 ? mapGridAxis(light_theme) : mapGrid(light_theme);
      cv::line(canvas, p0, p1, color, y == 0 ? 2 : 1, cv::LINE_AA);
    }
  }

  void drawGeom(cv::Mat & canvas, const MapGeom & geom) const
  {
    if (geom.type == MapGeomType::Box) {
      std::vector<cv::Point> points;
      points.reserve(geom.corners.size());
      for (const auto & corner : geom.corners) {
        points.push_back(worldToPixel(corner));
      }
      cv::fillConvexPoly(canvas, points, geom.fill, cv::LINE_AA);
      cv::polylines(canvas, points, true, geom.outline, 1, cv::LINE_AA);
      return;
    }

    const auto center = worldToPixel(geom.center);
    const int radius = std::max(2, static_cast<int>(std::lround(geom.radius * scale)));
    cv::circle(canvas, center, radius, geom.fill, cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, center, radius, geom.outline, 1, cv::LINE_AA);
  }

  void drawSavedPoints(cv::Mat & canvas, const MapUiState & ui_state) const
  {
    if (points.empty()) {
      return;
    }

    for (std::size_t i = 1; i < points.size(); ++i) {
      if (ui_state.route_patch_active && i == ui_state.route_patch_insert_index) {
        continue;
      }

      const auto p0 = worldToPixel({points[i - 1].x, points[i - 1].y});
      const auto p1 = worldToPixel({points[i].x, points[i].y});
      const bool fast_segment =
        points[i - 1].fast && points[i].fast &&
        points[i - 1].task_type == kTaskTypeNone && points[i].task_type == kTaskTypeNone;
      cv::line(
        canvas,
        p0,
        p1,
        fast_segment ? cv::Scalar(42, 58, 235) : cv::Scalar(40, 220, 220),
        fast_segment ? 3 : 2,
        cv::LINE_AA);
    }

    for (const auto & point : points) {
      const auto center = worldToPixel({point.x, point.y});
      cv::Scalar point_color = point.fast ? cv::Scalar(42, 58, 235) : cv::Scalar(40, 220, 220);
      if (point.task_type == kTaskTypePickup) {
        point_color = cv::Scalar(42, 58, 235);
      } else if (point.task_type == kTaskTypePlace) {
        point_color = cv::Scalar(58, 150, 235);
      }
      cv::circle(canvas, center, 8, cv::Scalar(20, 30, 35), cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, center, 6, point_color, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, center, 8, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);

      const auto label = std::to_string(point.id);
      cv::putText(
        canvas,
        label,
        cv::Point(center.x + 10, center.y - 8),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(245, 245, 245),
        2,
        cv::LINE_AA);
      cv::putText(
        canvas,
        label,
        cv::Point(center.x + 10, center.y - 8),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(20, 30, 35),
        1,
        cv::LINE_AA);
    }
  }

  void drawRoutePatch(cv::Mat & canvas, const MapUiState & ui_state) const
  {
    if (!ui_state.route_patch_active || points.empty()) {
      return;
    }

    std::vector<cv::Point> route_pixels;
    const auto insert_index = std::min(ui_state.route_patch_insert_index, points.size());
    if (insert_index > 0) {
      route_pixels.push_back(worldToPixel({points[insert_index - 1].x, points[insert_index - 1].y}));
    }
    for (const auto & point : ui_state.route_patch_points) {
      route_pixels.push_back(worldToPixel({point.x, point.y}));
    }
    if (!ui_state.route_patch_points.empty() && insert_index < points.size()) {
      route_pixels.push_back(worldToPixel({points[insert_index].x, points[insert_index].y}));
    }

    const cv::Scalar patch_line(48, 160, 48);
    for (std::size_t i = 1; i < route_pixels.size(); ++i) {
      drawDashedArrow(canvas, route_pixels[i - 1], route_pixels[i], patch_line, 2);
    }

    if (insert_index > 0) {
      const auto previous = worldToPixel({points[insert_index - 1].x, points[insert_index - 1].y});
      cv::circle(canvas, previous, 12, patch_line, 2, cv::LINE_AA);
    }
    if (insert_index < points.size()) {
      const auto next = worldToPixel({points[insert_index].x, points[insert_index].y});
      cv::circle(canvas, next, 12, patch_line, 2, cv::LINE_AA);
    }

    for (const auto & point : ui_state.route_patch_points) {
      const auto center = worldToPixel({point.x, point.y});
      cv::circle(canvas, center, 8, cv::Scalar(20, 30, 35), cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, center, 6, patch_line, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, center, 8, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    }
  }

  void drawDashedArrow(
    cv::Mat & canvas,
    cv::Point start,
    cv::Point end,
    cv::Scalar color,
    int thickness) const
  {
    const double dx = static_cast<double>(end.x - start.x);
    const double dy = static_cast<double>(end.y - start.y);
    const double length = std::hypot(dx, dy);
    if (length < 2.0) {
      return;
    }

    constexpr double dash = 14.0;
    constexpr double gap = 9.0;
    const double ux = dx / length;
    const double uy = dy / length;
    for (double offset = 0.0; offset < length; offset += dash + gap) {
      const double segment_end = std::min(length, offset + dash);
      const cv::Point p0(
        start.x + static_cast<int>(std::lround(ux * offset)),
        start.y + static_cast<int>(std::lround(uy * offset)));
      const cv::Point p1(
        start.x + static_cast<int>(std::lround(ux * segment_end)),
        start.y + static_cast<int>(std::lround(uy * segment_end)));
      cv::line(canvas, p0, p1, color, thickness, cv::LINE_AA);
    }

    const double arrow_start_distance = std::max(0.0, length - 24.0);
    const cv::Point arrow_start(
      start.x + static_cast<int>(std::lround(ux * arrow_start_distance)),
      start.y + static_cast<int>(std::lround(uy * arrow_start_distance)));
    cv::arrowedLine(canvas, arrow_start, end, color, thickness, cv::LINE_AA, 0, 0.32);
  }

  void drawPlanOrderBadge(cv::Mat & canvas, cv::Point center, int order) const
  {
    const std::string label = std::to_string(order);
    const int radius = 13;
    const cv::Point badge_center(center.x, center.y - 18);
    cv::circle(canvas, badge_center, radius + 2, cv::Scalar(20, 70, 35), cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, badge_center, radius, cv::Scalar(245, 255, 245), cv::FILLED, cv::LINE_AA);

    int baseline = 0;
    const auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.48, 1, &baseline);
    const cv::Point text_origin(
      badge_center.x - text_size.width / 2,
      badge_center.y + text_size.height / 2);
    cv::putText(
      canvas,
      label,
      text_origin,
      cv::FONT_HERSHEY_SIMPLEX,
      0.48,
      cv::Scalar(20, 70, 35),
      1,
      cv::LINE_AA);
  }

  void drawOptimizedPlan(cv::Mat & canvas, const MapUiState & ui_state) const
  {
    if (ui_state.mission_plan_display_mode == navigation::ui::MapPlanDisplayMode::Hidden) {
      return;
    }
    if (ui_state.mission_plan_points.size() < 2) {
      return;
    }

    const bool draw_arrows =
      ui_state.mission_plan_display_mode == navigation::ui::MapPlanDisplayMode::Full;
    int delivery_order = 1;
    for (std::size_t i = 1; i < ui_state.mission_plan_points.size(); ++i) {
      const auto & from = ui_state.mission_plan_points[i - 1];
      const auto & to = ui_state.mission_plan_points[i];
      const auto to_pixel = worldToPixel({to.x, to.y});
      const cv::Scalar color = to.loaded_segment_to_here ?
        cv::Scalar(42, 120, 28) :
        cv::Scalar(230, 120, 40);
      if (draw_arrows) {
        drawDashedArrow(
          canvas,
          worldToPixel({from.x, from.y}),
          to_pixel,
          color,
          2);
      }
      if (!to.loaded_segment_to_here) {
        drawPlanOrderBadge(canvas, to_pixel, delivery_order);
        ++delivery_order;
      }
    }
  }

  void drawRobot(cv::Mat & canvas, const navigation::RobotNavigationState & state) const
  {
    const auto center = worldToPixel({state.x, state.y});
    const int radius = std::max(6, static_cast<int>(std::lround(0.16 * scale)));
    cv::circle(canvas, center, radius + 2, cv::Scalar(245, 245, 245), cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, center, radius, cv::Scalar(40, 70, 235), cv::FILLED, cv::LINE_AA);

    const cv::Point heading_end(
      center.x + static_cast<int>(std::lround(std::cos(state.yaw) * radius * 2.4)),
      center.y - static_cast<int>(std::lround(std::sin(state.yaw) * radius * 2.4)));
    cv::arrowedLine(canvas, center, heading_end, cv::Scalar(255, 255, 255), 2, cv::LINE_AA, 0, 0.35);

    if (state.planar_speed > 0.03) {
      const Vec2 velocity_end{
        state.x + std::clamp(state.linear_x, -2.0, 2.0) * 0.5,
        state.y + std::clamp(state.linear_y, -2.0, 2.0) * 0.5,
      };
      cv::arrowedLine(
        canvas,
        center,
        worldToPixel(velocity_end),
        cv::Scalar(255, 180, 60),
        2,
        cv::LINE_AA,
        0,
        0.25);
    }
  }

  void drawStatus(
    cv::Mat & canvas,
    const navigation::RobotNavigationState & state,
    const MapUiState & ui_state) const
  {
    const bool light_theme = ui_state.light_theme;
    cv::rectangle(canvas, cv::Rect(10, 58, 410, 76), mapSurface(light_theme), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(10, 58, 410, 76), mapSurfaceBorder(light_theme), 1);

    char line1[160];
    char line2[160];
    std::snprintf(
      line1,
      sizeof(line1),
      "%s  x %.2f  y %.2f  yaw %.1f deg",
      state.source.c_str(),
      state.x,
      state.y,
      state.yaw * 180.0 / kPi);
    if (ui_state.cmd_vel_valid) {
      std::snprintf(
        line2,
        sizeof(line2),
        "cmd_vel: vx %.2f  vy %.2f  w %.2f",
        ui_state.cmd_vel_linear_x,
        ui_state.cmd_vel_linear_y,
        ui_state.cmd_vel_angular_z);
    } else {
      std::snprintf(line2, sizeof(line2), "cmd_vel: waiting");
    }

    cv::putText(canvas, line1, cv::Point(20, 86), cv::FONT_HERSHEY_SIMPLEX, 0.55, mapText(light_theme), 1, cv::LINE_AA);
    cv::putText(canvas, line2, cv::Point(20, 114), cv::FONT_HERSHEY_SIMPLEX, 0.55, mapMutedText(light_theme), 1, cv::LINE_AA);
  }

  int width;
  int height;
  double padding;
  double scale{1.0};
  double fit_scale{1.0};
  double view_left{0.0};
  double view_top{0.0};
  std::string scene_path;
  std::filesystem::path scene_dir;
  std::filesystem::path mesh_dir;
  Bounds bounds;
  std::vector<MapGeom> geoms;
  std::vector<MapPoint> points;
  cv::Mat image_background;
  std::map<std::string, std::pair<double, double>> hfields;
  std::map<std::string, MeshAsset> meshes;
  std::map<std::string, cv::Scalar> materials;
  navigation::ui::MapUiRenderer ui_renderer;
};

TopViewMap::TopViewMap(int width, int height, double padding)
: impl_(std::make_unique<Impl>(width, height, padding))
{
}

TopViewMap::~TopViewMap() = default;

void TopViewMap::load(const std::string & scene_path)
{
  impl_->load(scene_path);
}

cv::Mat TopViewMap::draw(const navigation::RobotNavigationState * state, const MapUiState & ui_state) const
{
  return impl_->draw(state, ui_state);
}

std::size_t TopViewMap::geomCount() const
{
  return impl_->geoms.size();
}

MapUiHit TopViewMap::hitTestUi(int pixel_x, int pixel_y, const MapUiState & ui_state) const
{
  return impl_->hitTestUi(pixel_x, pixel_y, ui_state);
}

bool TopViewMap::pixelToWorld(int pixel_x, int pixel_y, MapPoint & point) const
{
  return impl_->pixelToWorld(pixel_x, pixel_y, point);
}

int TopViewMap::hitTestPoint(int pixel_x, int pixel_y, int radius_px) const
{
  const int radius_sq = radius_px * radius_px;
  for (std::size_t i = 0; i < impl_->points.size(); ++i) {
    const auto center = impl_->worldToPixel({impl_->points[i].x, impl_->points[i].y});
    const int dx = pixel_x - center.x;
    const int dy = pixel_y - center.y;
    if (dx * dx + dy * dy <= radius_sq) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int TopViewMap::nearestPointIndex(int pixel_x, int pixel_y) const
{
  if (impl_->points.empty()) {
    return -1;
  }

  int nearest_index = -1;
  double nearest_distance_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < impl_->points.size(); ++i) {
    const auto center = impl_->worldToPixel({impl_->points[i].x, impl_->points[i].y});
    const double dx = static_cast<double>(pixel_x - center.x);
    const double dy = static_cast<double>(pixel_y - center.y);
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_index = static_cast<int>(i);
    }
  }

  return nearest_index;
}

bool TopViewMap::zoomAt(int pixel_x, int pixel_y, double factor)
{
  return impl_->zoomAt(pixel_x, pixel_y, factor);
}

bool TopViewMap::panBy(int delta_x, int delta_y)
{
  return impl_->panBy(delta_x, delta_y);
}

void TopViewMap::setPoints(const std::vector<MapPoint> & points)
{
  impl_->points = points;
}

void TopViewMap::addPoint(const MapPoint & point)
{
  impl_->points.push_back(point);
}

bool TopViewMap::setPointFast(std::size_t index, bool fast)
{
  if (index >= impl_->points.size()) {
    return false;
  }

  impl_->points[index].fast = fast;
  if (!fast) {
    impl_->points[index].task_type = kTaskTypeNone;
  }
  return true;
}

bool TopViewMap::togglePointFast(std::size_t index)
{
  if (index >= impl_->points.size()) {
    return false;
  }

  impl_->points[index].fast = !impl_->points[index].fast;
  if (!impl_->points[index].fast) {
    impl_->points[index].task_type = kTaskTypeNone;
  }
  return true;
}

bool TopViewMap::removePoint(std::size_t index)
{
  if (index >= impl_->points.size()) {
    return false;
  }

  impl_->points.erase(impl_->points.begin() + static_cast<std::ptrdiff_t>(index));
  for (std::size_t i = 0; i < impl_->points.size(); ++i) {
    impl_->points[i].id = static_cast<int>(i + 1);
  }
  return true;
}

bool TopViewMap::removeLastPoint()
{
  if (impl_->points.empty()) {
    return false;
  }

  impl_->points.pop_back();
  return true;
}

void TopViewMap::clearPoints()
{
  impl_->points.clear();
}

const std::vector<MapPoint> & TopViewMap::points() const
{
  return impl_->points;
}

}  // namespace navigation::maps
