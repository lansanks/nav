from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rname_arg = DeclareLaunchArgument("rname", default_value="blackW")
    scene_arg = DeclareLaunchArgument("scene", default_value="terrain")
    render_arg = DeclareLaunchArgument("render", default_value="true")
    render_rate_hz_arg = DeclareLaunchArgument("render_rate_hz", default_value="60.0")
    real_time_arg = DeclareLaunchArgument("real_time", default_value="true")
    publish_rate_hz_arg = DeclareLaunchArgument("publish_rate_hz", default_value="500.0")
    show_window_arg = DeclareLaunchArgument("show_window", default_value="true")
    points_file_arg = DeclareLaunchArgument("points_file", default_value="")
    cmd_vel_topic_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel")
    waypoint_tolerance_arg = DeclareLaunchArgument("waypoint_tolerance", default_value="0.20")
    max_linear_speed_arg = DeclareLaunchArgument("max_linear_speed", default_value="0.70")
    max_angular_speed_arg = DeclareLaunchArgument("max_angular_speed", default_value="1.80")
    fast_max_linear_speed_arg = DeclareLaunchArgument("fast_max_linear_speed", default_value="1.20")
    fast_max_angular_speed_arg = DeclareLaunchArgument("fast_max_angular_speed", default_value="2.20")
    constant_speed_linear_x_arg = DeclareLaunchArgument("constant_speed_linear_x", default_value="0.60")
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
    navigation_event_wait_seconds_arg = DeclareLaunchArgument(
        "navigation_event_wait_seconds",
        default_value="1.0",
    )
    rl_debug_key_topic_arg = DeclareLaunchArgument("rl_debug_key_topic", default_value="/rl_sim/debug_key")
    rl_policy_config_topic_arg = DeclareLaunchArgument(
        "rl_policy_config_topic",
        default_value="/rl_sim/policy_config",
    )

    rname = LaunchConfiguration("rname")
    scene = LaunchConfiguration("scene")
    render = LaunchConfiguration("render")
    render_rate_hz = LaunchConfiguration("render_rate_hz")
    real_time = LaunchConfiguration("real_time")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    show_window = LaunchConfiguration("show_window")
    points_file = LaunchConfiguration("points_file")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    waypoint_tolerance = LaunchConfiguration("waypoint_tolerance")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    max_angular_speed = LaunchConfiguration("max_angular_speed")
    fast_max_linear_speed = LaunchConfiguration("fast_max_linear_speed")
    fast_max_angular_speed = LaunchConfiguration("fast_max_angular_speed")
    constant_speed_linear_x = LaunchConfiguration("constant_speed_linear_x")
    ui_size = LaunchConfiguration("ui_size")
    map_width_px = LaunchConfiguration("map_width_px")
    map_height_px = LaunchConfiguration("map_height_px")
    map_padding_px = LaunchConfiguration("map_padding_px")
    panel_collapsed = LaunchConfiguration("panel_collapsed")
    race_logic = LaunchConfiguration("race_logic")
    mission_task_radius = LaunchConfiguration("mission_task_radius")
    mission_resume_event = LaunchConfiguration("mission_resume_event")
    mission_pickup_resume_event = LaunchConfiguration("mission_pickup_resume_event")
    mission_place_resume_event = LaunchConfiguration("mission_place_resume_event")
    arm_mission_service = LaunchConfiguration("arm_mission_service")
    navigation_arm_event_service = LaunchConfiguration("navigation_arm_event_service")
    mission_arm_retry_period = LaunchConfiguration("mission_arm_retry_period")
    navigation_event_wait_seconds = LaunchConfiguration("navigation_event_wait_seconds")
    rl_debug_key_topic = LaunchConfiguration("rl_debug_key_topic")
    rl_policy_config_topic = LaunchConfiguration("rl_policy_config_topic")

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        output="screen",
        parameters=[
            {
                "robot_name": ParameterValue(rname, value_type=str),
                "gazebo_model_name": ParameterValue([rname, "_gazebo"], value_type=str),
            }
        ],
    )

    mujoco_node = Node(
        package="mujoco_runner",
        executable="mm",
        name="mujoco_node",
        output="screen",
        parameters=[
            {
                "rname": ParameterValue(rname, value_type=str),
                "scene": ParameterValue(scene, value_type=str),
                "publish_odom": True,
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "render": ParameterValue(render, value_type=bool),
                "render_rate_hz": ParameterValue(render_rate_hz, value_type=float),
                "real_time": ParameterValue(real_time, value_type=bool),
            }
        ],
    )

    midware_node = Node(
        package="midware",
        executable="middleware",
    )

    navigation_node = Node(
        package="navigation",
        executable="navigation_map",
        name="navigation_map",
        output="screen",
        parameters=[
            {
                "source": "simulation",
                "robot_name": ParameterValue(rname, value_type=str),
                "scene": ParameterValue(scene, value_type=str),
                "sim_odom_topic": "/odom",
                "show_window": ParameterValue(show_window, value_type=bool),
                "points_file": ParameterValue(points_file, value_type=str),
                "cmd_vel_topic": ParameterValue(cmd_vel_topic, value_type=str),
                "waypoint_tolerance": ParameterValue(waypoint_tolerance, value_type=float),
                "max_linear_speed": ParameterValue(max_linear_speed, value_type=float),
                "max_angular_speed": ParameterValue(max_angular_speed, value_type=float),
                "fast_max_linear_speed": ParameterValue(fast_max_linear_speed, value_type=float),
                "fast_max_angular_speed": ParameterValue(fast_max_angular_speed, value_type=float),
                "constant_speed_linear_x": ParameterValue(constant_speed_linear_x, value_type=float),
                "ui_size": ParameterValue(ui_size, value_type=int),
                "map_width_px": ParameterValue(map_width_px, value_type=int),
                "map_height_px": ParameterValue(map_height_px, value_type=int),
                "map_padding_px": ParameterValue(map_padding_px, value_type=float),
                "panel_collapsed": ParameterValue(panel_collapsed, value_type=bool),
                "race_logic": ParameterValue(race_logic, value_type=str),
                "mission_task_radius": ParameterValue(mission_task_radius, value_type=float),
                "mission_resume_event": ParameterValue(mission_resume_event, value_type=str),
                "mission_pickup_resume_event": ParameterValue(
                    mission_pickup_resume_event,
                    value_type=str,
                ),
                "mission_place_resume_event": ParameterValue(
                    mission_place_resume_event,
                    value_type=str,
                ),
                "arm_mission_service": ParameterValue(arm_mission_service, value_type=str),
                "navigation_arm_event_service": ParameterValue(navigation_arm_event_service, value_type=str),
                "mission_arm_retry_period": ParameterValue(mission_arm_retry_period, value_type=float),
                "navigation_event_wait_seconds": ParameterValue(
                    navigation_event_wait_seconds,
                    value_type=float,
                ),
                "rl_debug_key_topic": ParameterValue(rl_debug_key_topic, value_type=str),
                "rl_policy_config_topic": ParameterValue(rl_policy_config_topic, value_type=str),
            }
        ],
    )

    return LaunchDescription(
        [
            rname_arg,
            scene_arg,
            render_arg,
            render_rate_hz_arg,
            real_time_arg,
            publish_rate_hz_arg,
            show_window_arg,
            points_file_arg,
            cmd_vel_topic_arg,
            waypoint_tolerance_arg,
            max_linear_speed_arg,
            max_angular_speed_arg,
            fast_max_linear_speed_arg,
            fast_max_angular_speed_arg,
            constant_speed_linear_x_arg,
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
            navigation_event_wait_seconds_arg,
            rl_debug_key_topic_arg,
            rl_policy_config_topic_arg,
            param_node,
            mujoco_node,
            midware_node,
            navigation_node,
        ]
    )
