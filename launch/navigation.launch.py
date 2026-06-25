from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_executable_arg = DeclareLaunchArgument("navigation_executable", default_value="navigation_map")
    node_role_arg = DeclareLaunchArgument("node_role", default_value="standalone")
    source_arg = DeclareLaunchArgument("source", default_value="simulation")
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="blackW")
    scene_arg = DeclareLaunchArgument("scene", default_value="terrain")
    sim_odom_topic_arg = DeclareLaunchArgument("sim_odom_topic", default_value="/odom")
    radar_odom_topic_arg = DeclareLaunchArgument("radar_odom_topic", default_value="/Odometry")
    radar_calibration_topic_arg = DeclareLaunchArgument("radar_calibration_topic", default_value="/Odometry")
    navigation_command_topic_arg = DeclareLaunchArgument(
        "navigation_command_topic",
        default_value="/navigation/command",
    )
    navigation_status_topic_arg = DeclareLaunchArgument(
        "navigation_status_topic",
        default_value="/navigation/status",
    )
    navigation_state_topic_arg = DeclareLaunchArgument(
        "navigation_state_topic",
        default_value="/navigation/state",
    )
    radar_calibration_file_arg = DeclareLaunchArgument(
        "radar_calibration_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("navigation"),
                "config",
                "calibration",
                "cali_params",
                "radar_points__to__cali505.yaml",
            ]
        ),
    )
    show_window_arg = DeclareLaunchArgument("show_window", default_value="true")
    points_file_arg = DeclareLaunchArgument("points_file", default_value="")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    waypoint_tolerance_arg = DeclareLaunchArgument("waypoint_tolerance", default_value="0.20")
    max_linear_speed_arg = DeclareLaunchArgument("max_linear_speed", default_value="0.70")
    max_angular_speed_arg = DeclareLaunchArgument("max_angular_speed", default_value="1.80")
    fast_max_linear_speed_arg = DeclareLaunchArgument("fast_max_linear_speed", default_value="1.20")
    fast_max_angular_speed_arg = DeclareLaunchArgument("fast_max_angular_speed", default_value="2.20")
    ui_size_arg = DeclareLaunchArgument("ui_size", default_value="10")
    map_width_px_arg = DeclareLaunchArgument("map_width_px", default_value="0")
    map_height_px_arg = DeclareLaunchArgument("map_height_px", default_value="0")
    map_padding_px_arg = DeclareLaunchArgument("map_padding_px", default_value="20.0")
    panel_collapsed_arg = DeclareLaunchArgument("panel_collapsed", default_value="false")
    race_logic_arg = DeclareLaunchArgument("race_logic", default_value="obstacle")
    mission_task_radius_arg = DeclareLaunchArgument("mission_task_radius", default_value="0.40")
    mission_resume_event_arg = DeclareLaunchArgument("mission_resume_event", default_value="completed")
    mission_pickup_resume_event_arg = DeclareLaunchArgument(
        "mission_pickup_resume_event",
        default_value="completed",
    )
    mission_place_resume_event_arg = DeclareLaunchArgument(
        "mission_place_resume_event",
        default_value="completed",
    )
    arm_mission_service_arg = DeclareLaunchArgument("arm_mission_service", default_value="/arm/mission_event")
    navigation_arm_event_service_arg = DeclareLaunchArgument(
        "navigation_arm_event_service",
        default_value="/navigation/arm_event",
    )
    mission_arm_retry_period_arg = DeclareLaunchArgument("mission_arm_retry_period", default_value="1.0")

    navigation_node = Node(
        package="navigation",
        executable=LaunchConfiguration("navigation_executable"),
        output="screen",
        parameters=[
            {
                "node_role": ParameterValue(LaunchConfiguration("node_role"), value_type=str),
                "source": ParameterValue(LaunchConfiguration("source"), value_type=str),
                "robot_name": ParameterValue(LaunchConfiguration("robot_name"), value_type=str),
                "scene": ParameterValue(LaunchConfiguration("scene"), value_type=str),
                "sim_odom_topic": ParameterValue(LaunchConfiguration("sim_odom_topic"), value_type=str),
                "radar_odom_topic": ParameterValue(LaunchConfiguration("radar_odom_topic"), value_type=str),
                "radar_calibration_topic": ParameterValue(LaunchConfiguration("radar_calibration_topic"), value_type=str),
                "navigation_command_topic": ParameterValue(
                    LaunchConfiguration("navigation_command_topic"),
                    value_type=str,
                ),
                "navigation_status_topic": ParameterValue(
                    LaunchConfiguration("navigation_status_topic"),
                    value_type=str,
                ),
                "navigation_state_topic": ParameterValue(
                    LaunchConfiguration("navigation_state_topic"),
                    value_type=str,
                ),
                "radar_calibration_file": ParameterValue(LaunchConfiguration("radar_calibration_file"), value_type=str),
                "show_window": ParameterValue(LaunchConfiguration("show_window"), value_type=bool),
                "points_file": ParameterValue(LaunchConfiguration("points_file"), value_type=str),
                "cmd_vel_topic": ParameterValue(LaunchConfiguration("cmd_vel_topic"), value_type=str),
                "waypoint_tolerance": ParameterValue(LaunchConfiguration("waypoint_tolerance"), value_type=float),
                "max_linear_speed": ParameterValue(LaunchConfiguration("max_linear_speed"), value_type=float),
                "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
                "fast_max_linear_speed": ParameterValue(LaunchConfiguration("fast_max_linear_speed"), value_type=float),
                "fast_max_angular_speed": ParameterValue(LaunchConfiguration("fast_max_angular_speed"), value_type=float),
                "ui_size": ParameterValue(LaunchConfiguration("ui_size"), value_type=int),
                "map_width_px": ParameterValue(LaunchConfiguration("map_width_px"), value_type=int),
                "map_height_px": ParameterValue(LaunchConfiguration("map_height_px"), value_type=int),
                "map_padding_px": ParameterValue(LaunchConfiguration("map_padding_px"), value_type=float),
                "panel_collapsed": ParameterValue(LaunchConfiguration("panel_collapsed"), value_type=bool),
                "race_logic": ParameterValue(LaunchConfiguration("race_logic"), value_type=str),
                "mission_task_radius": ParameterValue(
                    LaunchConfiguration("mission_task_radius"),
                    value_type=float,
                ),
                "mission_resume_event": ParameterValue(
                    LaunchConfiguration("mission_resume_event"),
                    value_type=str,
                ),
                "mission_pickup_resume_event": ParameterValue(
                    LaunchConfiguration("mission_pickup_resume_event"),
                    value_type=str,
                ),
                "mission_place_resume_event": ParameterValue(
                    LaunchConfiguration("mission_place_resume_event"),
                    value_type=str,
                ),
                "arm_mission_service": ParameterValue(
                    LaunchConfiguration("arm_mission_service"),
                    value_type=str,
                ),
                "navigation_arm_event_service": ParameterValue(
                    LaunchConfiguration("navigation_arm_event_service"),
                    value_type=str,
                ),
                "mission_arm_retry_period": ParameterValue(
                    LaunchConfiguration("mission_arm_retry_period"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            navigation_executable_arg,
            node_role_arg,
            source_arg,
            robot_name_arg,
            scene_arg,
            sim_odom_topic_arg,
            radar_odom_topic_arg,
            radar_calibration_topic_arg,
            navigation_command_topic_arg,
            navigation_status_topic_arg,
            navigation_state_topic_arg,
            radar_calibration_file_arg,
            show_window_arg,
            points_file_arg,
            cmd_vel_topic_arg,
            waypoint_tolerance_arg,
            max_linear_speed_arg,
            max_angular_speed_arg,
            fast_max_linear_speed_arg,
            fast_max_angular_speed_arg,
            ui_size_arg,
            map_width_px_arg,
            map_height_px_arg,
            map_padding_px_arg,
            panel_collapsed_arg,
            race_logic_arg,
            mission_task_radius_arg,
            mission_resume_event_arg,
            mission_pickup_resume_event_arg,
            mission_place_resume_event_arg,
            arm_mission_service_arg,
            navigation_arm_event_service_arg,
            mission_arm_retry_period_arg,
            navigation_node,
        ]
    )
