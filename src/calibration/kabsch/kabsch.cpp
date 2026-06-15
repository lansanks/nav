#include "calibration/kabsch.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "opencv2/core.hpp"

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

double centeredAreaScore(const std::vector<navigation::maps::MapPoint> & points)
{
  if (points.size() < 3) {
    return 0.0;
  }

  double cx = 0.0;
  double cy = 0.0;
  for (const auto & point : points) {
    cx += point.x;
    cy += point.y;
  }
  cx /= static_cast<double>(points.size());
  cy /= static_cast<double>(points.size());

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const auto & point : points) {
    const double x = point.x - cx;
    const double y = point.y - cy;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  return std::abs(xx * yy - xy * xy);
}

}  // namespace

bool computeKabsch(
  const std::vector<navigation::maps::MapPoint> & radar_points,
  const std::vector<navigation::maps::MapPoint> & mujoco_points,
  KabschResult & result,
  std::string * error_message)
{
  if (radar_points.size() != mujoco_points.size()) {
    setError(error_message, "point counts do not match");
    return false;
  }
  if (radar_points.size() < 2) {
    setError(error_message, "at least 2 point pairs are required");
    return false;
  }

  const auto count = static_cast<double>(radar_points.size());
  cv::Point2d radar_center(0.0, 0.0);
  cv::Point2d mujoco_center(0.0, 0.0);
  for (std::size_t i = 0; i < radar_points.size(); ++i) {
    radar_center.x += radar_points[i].x;
    radar_center.y += radar_points[i].y;
    mujoco_center.x += mujoco_points[i].x;
    mujoco_center.y += mujoco_points[i].y;
  }
  radar_center.x /= count;
  radar_center.y /= count;
  mujoco_center.x /= count;
  mujoco_center.y /= count;

  cv::Mat H = cv::Mat::zeros(2, 2, CV_64F);
  for (std::size_t i = 0; i < radar_points.size(); ++i) {
    const double px = radar_points[i].x - radar_center.x;
    const double py = radar_points[i].y - radar_center.y;
    const double qx = mujoco_points[i].x - mujoco_center.x;
    const double qy = mujoco_points[i].y - mujoco_center.y;
    H.at<double>(0, 0) += px * qx;
    H.at<double>(0, 1) += px * qy;
    H.at<double>(1, 0) += py * qx;
    H.at<double>(1, 1) += py * qy;
  }

  cv::SVD svd(H, cv::SVD::FULL_UV);
  cv::Mat V = svd.vt.t();
  cv::Mat R = V * svd.u.t();
  if (cv::determinant(R) < 0.0) {
    V.at<double>(0, 1) *= -1.0;
    V.at<double>(1, 1) *= -1.0;
    R = V * svd.u.t();
  }

  result = KabschResult{};
  result.r00 = R.at<double>(0, 0);
  result.r01 = R.at<double>(0, 1);
  result.r10 = R.at<double>(1, 0);
  result.r11 = R.at<double>(1, 1);
  result.tx = mujoco_center.x - (result.r00 * radar_center.x + result.r01 * radar_center.y);
  result.ty = mujoco_center.y - (result.r10 * radar_center.x + result.r11 * radar_center.y);
  result.yaw_offset = std::atan2(result.r10, result.r00);
  result.unstable = centeredAreaScore(radar_points) < 1e-9 || centeredAreaScore(mujoco_points) < 1e-9;
  result.errors.reserve(radar_points.size());

  double error_sum = 0.0;
  for (std::size_t i = 0; i < radar_points.size(); ++i) {
    const double predicted_x = result.r00 * radar_points[i].x + result.r01 * radar_points[i].y + result.tx;
    const double predicted_y = result.r10 * radar_points[i].x + result.r11 * radar_points[i].y + result.ty;
    const double error = std::hypot(mujoco_points[i].x - predicted_x, mujoco_points[i].y - predicted_y);
    result.errors.push_back(error);
    error_sum += error;
    result.max_error = std::max(result.max_error, error);
  }
  result.mean_error = error_sum / count;
  return true;
}

}  // namespace navigation::calibration
