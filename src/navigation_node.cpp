#include <exception>
#include <memory>

#include "app/navigation_map_node.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<navigation::app::NavigationMapNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_map"), "%s", error.what());
  }
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
