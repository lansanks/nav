#ifndef NAVIGATION_APP_NAVIGATION_NODE_CONTEXT_HPP_
#define NAVIGATION_APP_NAVIGATION_NODE_CONTEXT_HPP_

#include <chrono>
#include <cstdint>
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
#include "navigation/srv/mission_command.hpp"
#include "params/navigation_params.hpp"
#include "std_msgs/msg/string.hpp"
#include "ui/navigation_ui_state.hpp"

namespace navigation::app
{

struct NavigationNodeContext
{
  std::string window_name;
  std::string robot_name;
  std::string current_map_file;
  std::string points_file;
  std::string ui_state_file;
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
  std::vector<int> dropdown_marked_indices;
  std::vector<int> dropdown_marked_order;
  std::vector<std::string> pending_merge_point_paths;
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
  navigation::ui::MapPlanDisplayMode mission_plan_display_mode{
    navigation::ui::MapPlanDisplayMode::Full};
  bool radar_popup_active{false};
  bool radar_result_pending{false};
  bool radar_save_file_confirmed{false};
  bool remote_control{false};
  bool remote_navigation_active{false};
  bool remote_navigation_complete{false};
  std::size_t remote_navigation_target_index{0};
  std::size_t remote_navigation_point_count{0};
  bool cmd_vel_valid{false};
  double cmd_vel_linear_x{0.0};
  double cmd_vel_linear_y{0.0};
  double cmd_vel_angular_z{0.0};
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
  std::string mission_pickup_resume_event{"completed"};
  std::string mission_place_resume_event{"completed"};
  std::string arm_mission_service{"/arm/mission_event"};
  std::string navigation_arm_event_service{"/navigation/arm_event"};
  double mission_arm_retry_period{1.0};
  bool settings_popup_active{false};
  int settings_selected_index{0};
  bool settings_editing{false};
  std::string settings_edit_text;
  std::string mission_slot_categories_text;
  std::string mission_high_score_category_text{"-1"};
  std::string mission_high_score_priority_text{"0"};
  std::string mission_cost_budget_text{"999.0"};
  std::string mission_alpha_text{"1.0"};
  std::string mission_beta_text{"0.3"};
  std::string mission_eta_text{"0.4"};
  std::string mission_g_pick_place_text{"0.0"};
  std::string mission_storage_near_distance_text{"0.40"};
  std::string mission_storage_far_distance_text{"0.50"};
  std::string mission_return_near_distance_text{"0.40"};
  std::string mission_return_far_distance_text{"0.50"};
  std::string mission_plan_summary;
  std::vector<navigation::ui::MapPlanPoint> mission_plan_points;
  bool route_patch_active{false};
  std::size_t route_patch_insert_index{0};
  std::vector<navigation::maps::MapPoint> route_patch_original_points;
  std::vector<navigation::maps::MapPoint> route_patch_points;
  std::size_t event_label_edit_index{0};
  bool event_label_edit_active{false};
  std::size_t segment_speed_edit_target_index{0};
  bool segment_speed_edit_active{false};
  int segment_speed_selected_index{0};
  bool segment_speed_field_editing{false};
  std::string segment_speed_edit_text;
  int segment_speed_last_click_index{-1};
  std::chrono::steady_clock::time_point segment_speed_last_click_time;
  bool segment_speed_constant_mode{true};
  std::uint8_t segment_speed_level{3};
  double segment_speed_linear_x{0.0};
  double segment_speed_max_angular_speed{0.0};
  double segment_speed_k_alpha{0.0};
  double segment_speed_k_beta{0.0};
  std::string segment_speed_title;
  double navigation_event_wait_seconds{1.0};
  std::string rl_debug_key_topic{"/rl_sim/debug_key"};
  std::string rl_policy_config_topic{"/rl_sim/policy_config"};
  std::vector<bool> navigation_event_triggered;
  bool navigation_event_wait_active{false};
  std::chrono::steady_clock::time_point navigation_event_wait_until;

  struct MissionTaskState
  {
    std::size_t point_index{0};
    int point_id{0};
    std::uint8_t task_type{navigation::maps::kTaskTypeNone};
    bool triggered{false};
    bool ack{false};
    bool grabbed{false};
    bool placed{false};
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
  rclcpp::Client<navigation::srv::StringCommand>::SharedPtr set_radar_calibration_client;
  rclcpp::Client<navigation::srv::MissionCommand>::SharedPtr arm_mission_client;

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
