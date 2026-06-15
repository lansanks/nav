#ifndef NAVIGATION_APP_NAVIGATION_NODE_CONTEXT_HPP_
#define NAVIGATION_APP_NAVIGATION_NODE_CONTEXT_HPP_

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>

#include "controller.hpp"
#include "interface.hpp"
#include "calibration/radar_calibration.hpp"
#include "interface/radar_pose_listener.hpp"
#include "keyboards/navigation_input_handler.hpp"
#include "maps/top_view_map.hpp"
#include "navigation/srv/set_waypoints.hpp"
#include "navigation/srv/set_controller_config.hpp"
#include "navigation/srv/start_navigation.hpp"
#include "navigation/srv/stop_navigation.hpp"
#include "navigation/srv/string_command.hpp"
#include "params/navigation_params.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ui/navigation_ui_state.hpp"

namespace navigation::app
{

struct NavigationNodeContext
{
  std::string window_name;
  std::string robot_name;
  std::string current_map_file;
  std::string points_file;
  std::string cmd_vel_topic;
  std::string radar_topic{"/Odometry"};
  std::string selected_controller_name;
  std::string navigation_status;
  std::string status_message;
  std::string input_label;
  std::string input_text;
  navigation::keyboards::TextInputMode input_mode{navigation::keyboards::TextInputMode::None};
  navigation::ui::MapDropdownMode dropdown_mode{navigation::ui::MapDropdownMode::None};
  std::vector<std::string> dropdown_paths;
  std::vector<std::string> dropdown_labels;
  std::vector<std::string> controller_names;
  std::vector<navigation::params::ParamField> param_fields;
  navigation::ui::OnlineParamsSession params_session;
  int dropdown_selected_index{-1};
  int map_width_px{678};
  int map_height_px{420};
  int panel_scroll_px{0};
  bool panel_collapsed{false};
  bool light_theme{true};
  bool fullscreen{false};
  bool fullscreen_dirty{false};
  bool radar_popup_active{false};
  bool radar_result_pending{false};
  bool radar_save_file_confirmed{false};
  bool remote_control{false};
  bool remote_navigation_active{false};
  bool remote_navigation_complete{false};
  std::size_t remote_navigation_target_index{0};
  std::size_t remote_navigation_point_count{0};
  std::string radar_data_file;
  std::string radar_points_file;
  navigation::RobotNavigationState radar_latest_state;
  navigation::RobotNavigationState remote_latest_state;
  navigation::calibration::KabschResult radar_pending_result;
  bool show_window{true};
  std::unique_ptr<navigation::maps::TopViewMap> map;
  std::unique_ptr<navigation::NavigationInterface> interface;
  std::unique_ptr<navigation::NavigationController> controller;
  navigation::RadarPoseListener radar_listener;
  navigation::ControllerConfig controller_config;
  std::string race_logic{"obstacle"};
  double mission_task_radius{0.40};
  std::string mission_resume_event{"completed"};
  std::string arm_mission_service{"/arm/mission_event"};
  std::string navigation_arm_event_service{"/navigation/arm_event"};
  double mission_arm_retry_period{1.0};

  struct MissionTaskState
  {
    std::size_t point_index{0};
    int point_id{0};
    bool triggered{false};
    bool ack{false};
    bool grabbed{false};
    bool completed{false};
  };

  std::vector<MissionTaskState> mission_tasks;
  bool mission_paused{false};
  bool mission_arrived_request_pending{false};
  std::size_t mission_current_task{0};
  std::chrono::steady_clock::time_point mission_last_arrived_send;

  // Service clients for remote_ui → core communication
  rclcpp::Client<navigation::srv::SetWaypoints>::SharedPtr set_waypoints_client;
  rclcpp::Client<navigation::srv::SetControllerConfig>::SharedPtr set_config_client;
  rclcpp::Client<navigation::srv::StartNavigation>::SharedPtr start_client;
  rclcpp::Client<navigation::srv::StopNavigation>::SharedPtr stop_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_mission_client;

  // Heartbeat monitoring (remote_ui mode)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_subscription;
  rclcpp::Time last_heartbeat_time;
  bool core_connected{false};
  static constexpr double HEARTBEAT_TIMEOUT_S = 3.0;

  // Operation timeout tracking (remote_ui mode)
  std::chrono::steady_clock::time_point pending_op_start;
  static constexpr double OP_TIMEOUT_S = 3.0;
};

}  // namespace navigation::app

#endif  // NAVIGATION_APP_NAVIGATION_NODE_CONTEXT_HPP_
