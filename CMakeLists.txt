cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
find_package(rosidl_default_generators REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)

# Generate ROS 2 interfaces (messages and services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapPoint.msg"
  "srv/SetWaypoints.srv"
  "srv/SetControllerConfig.srv"
  "srv/StartNavigation.srv"
  "srv/StopNavigation.srv"
  "srv/StringCommand.srv"
  "srv/MissionCommand.srv"
  DEPENDENCIES
  std_msgs
)

set(NAVIGATION_UI_SOURCES
  src/app/navigation_map_node.cpp
  src/app/navigation_mouse_controller.cpp
  src/app/navigation_points_workflow.cpp
  src/app/navigation_runtime.cpp
  src/app/navigation_ui_coordinator.cpp
  src/calibration/kabsch/kabsch.cpp
  src/calibration/radar_calibration.cpp
  src/controller/sequential_waypoint_controller.cpp
  src/keyboards/navigation_input_handler.cpp
  src/keyboards/navigation_keys.cpp
  src/ui/map_ui_renderer.cpp
  src/ui/navigation_ui_state.cpp
  src/ui/window_scroll_controller.cpp
  src/params/navigation_params.cpp
  src/maps/navigation_map_helpers.cpp
  src/maps/point_store.cpp
  src/optim/task_order_planner.cpp
  src/maps/top_view_map.cpp
  src/interface/simulation_interface.cpp
  src/interface/radar_interface.cpp
  src/interface/radar_pose_listener.cpp
)

set(NAVIGATION_CORE_SOURCES
  src/app/navigation_runtime.cpp
  src/calibration/kabsch/kabsch.cpp
  src/calibration/radar_calibration.cpp
  src/controller/sequential_waypoint_controller.cpp
  src/keyboards/navigation_keys.cpp
  src/ui/map_ui_renderer.cpp
  src/ui/navigation_ui_state.cpp
  src/params/navigation_params.cpp
  src/maps/navigation_map_helpers.cpp
  src/maps/point_store.cpp
  src/maps/top_view_map.cpp
  src/interface/simulation_interface.cpp
  src/interface/radar_interface.cpp
  src/interface/radar_pose_listener.cpp
)

add_executable(navigation_map
  src/navigation_node.cpp
  ${NAVIGATION_UI_SOURCES}
)

add_executable(navigation_remote_ui
  src/navigation_remote_ui_node.cpp
  ${NAVIGATION_UI_SOURCES}
)

add_executable(navigation_core
  src/navigation_core_node.cpp
  ${NAVIGATION_CORE_SOURCES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

foreach(target navigation_map navigation_remote_ui navigation_core)
  target_include_directories(${target}
    PRIVATE
      include
      ${OpenCV_INCLUDE_DIRS}
  )

  ament_target_dependencies(${target}
    ament_index_cpp
    geometry_msgs
    nav_msgs
    rclcpp
    std_msgs
    std_srvs
  )

  target_link_libraries(${target}
    ${OpenCV_LIBS}
    Qt5::Widgets
    PkgConfig::TINYXML2
    PkgConfig::GTK3
    ${cpp_typesupport_target}
  )
endforeach()

install(TARGETS
  navigation_map
  navigation_remote_ui
  navigation_core
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigation_launcher
  scripts/navigation_core_launcher
  scripts/navigation_ui_launcher
  scripts/mock_arm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(optim PACKAGE_DIR src/optim)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY arm_integration
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
