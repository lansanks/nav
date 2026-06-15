#include <exception>
#include <memory>

#include "app/navigation_map_node.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<navigation::app::NavigationMapNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_remote_ui"), "%s", error.what());
  }
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
