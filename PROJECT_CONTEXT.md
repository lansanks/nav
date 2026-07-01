# Navigation Project Context

This file is a quick handoff note for future Codex conversations. Read it first before changing this repository.

## Project Summary

`navigation` is a ROS 2 `ament_cmake` package for the `blackW` top-view navigation workflow. It provides:

- an OpenCV HighGUI map UI for editing waypoints, selecting maps, tuning parameters, radar calibration, and starting/stopping navigation;
- a waypoint-following controller that publishes `geometry_msgs/Twist`;
- pose-source adapters for simulation odometry and radar odometry;
- a split-machine mode with a remote UI and an onboard core node;
- mission-race support for pickup/place tasks and mechanical-arm event synchronization.

The package is mostly C++17. Python is used for launch files, Tkinter launcher scripts, and a Python mirror of the task-order planner.

## Build And Main Targets

Primary files:

- `CMakeLists.txt`: declares ROS dependencies, generated msg/srv interfaces, source groups, and executables.
- `package.xml`: ROS package metadata and dependencies.

Built executables:

- `navigation_map`: standalone GUI plus controller. Entry: `src/navigation_node.cpp`.
- `navigation_remote_ui`: remote UI node. Entry: `src/navigation_remote_ui_node.cpp`.
- `navigation_core`: onboard/core navigation node. Entry: `src/navigation_core_node.cpp`.

Typical build from the workspace root:

```bash
colcon build --packages-select navigation
```

## Runtime Modes

### Standalone

`navigation_map` with `node_role:=standalone` owns both the UI and the controller. It subscribes to a local pose source, updates the controller every timer tick, and publishes velocity commands.

Important flow:

```text
NavigationMapNode
  -> NavigationRuntime::updateNavigationController()
  -> NavigationController::update()
  -> /cmd_vel publisher
```

### Remote UI And Core

`navigation_remote_ui` runs `NavigationMapNode` with `node_role:=remote_ui`. It does not create a pose interface or publish velocity commands. Instead it sends service requests to `navigation_core`.

`navigation_core` owns the real pose source, controller, velocity publisher, heartbeat publisher, and service servers.

Important communication:

- UI -> core services:
  - `/navigation/set_waypoints`
  - `/navigation/set_config`
  - `/navigation/start`
  - `/navigation/stop`
  - `/navigation/set_radar_calibration`
- Core -> UI topics:
  - `/navigation/state` (`nav_msgs/Odometry`)
  - `/navigation/status` (`std_msgs/String`, semicolon-separated fields)
  - `/navigation/heartbeat` (`std_msgs/String`, 1 Hz)

The remote UI marks the core disconnected if heartbeat is absent for more than `NavigationNodeContext::HEARTBEAT_TIMEOUT_S` seconds.

## Key Architecture

### Shared State

`include/app/navigation_node_context.hpp` defines `NavigationNodeContext`, the shared mutable state passed between the node, runtime, UI coordinator, mouse controller, and point workflow.

This struct contains:

- current map, point file, UI state, selected controller, status strings;
- controller config and editable parameter fields;
- local/remote navigation state;
- radar calibration popup state;
- mission task state and arm-service clients;
- remote service clients, heartbeat status, and operation timeout tracking.

Because this context is shared widely, check all users before renaming fields or changing their meaning.

### Runtime Logic

`include/app/navigation_runtime.hpp` and `src/app/navigation_runtime.cpp` contain the core navigation behavior shared by standalone and remote UI modes.

Important responsibilities:

- apply controller config;
- start/stop navigation;
- sync route changes to the controller or remote core;
- update controller from pose state;
- publish zero velocity when stopped or waiting;
- handle mission pickup/place pauses and arm events;
- trigger RL/debug events from waypoint `event_label`;
- send async service requests in remote mode.

The important branch is `context_.remote_control`:

- `false`: operate local controller directly.
- `true`: send service requests and wait for core status/heartbeat.

### Controller

Controller interface: `include/controller.hpp`.

Current implementation: `src/controller/sequential_waypoint_controller.cpp`.

Only one controller is registered: `Sequential Waypoint`.

Behavior:

- follows ordered `maps::MapPoint` waypoints;
- advances when within `waypoint_tolerance`;
- supports normal P-style velocity control;
- supports fast segments when both adjacent points are `fast`;
- supports constant-speed segments when both adjacent points are `constant_speed`;
- supports per-segment custom speed and angular gains;
- chooses a start index near the current pose when starting.

Mission mode deliberately clears fast/constant-speed behavior before points reach the controller.

### Pose Interfaces

Interface definition: `include/interface.hpp`.

Implementations:

- `src/interface/simulation_interface.cpp`: subscribes to `sim_odom_topic`, default `/odom`.
- `src/interface/radar_interface.cpp`: subscribes to `radar_odom_topic`, default `/Odometry`, and optionally applies a 2D calibration transform from a YAML-like file.

`RobotNavigationState` is the internal normalized pose/velocity representation.

### Map And Point Storage

Important files:

- `include/maps/top_view_map.hpp`
- `src/maps/top_view_map.cpp`
- `include/maps/point_store.hpp`
- `src/maps/point_store.cpp`
- `include/maps/navigation_map_helpers.hpp`
- `src/maps/navigation_map_helpers.cpp`

`TopViewMap` loads a scene or image map, converts pixels to world coordinates, draws the map and UI overlay, supports zoom/pan/hit-testing, and owns the current `std::vector<MapPoint>`.

Map files live under `config/maps/`.

Point files live under `config/points/*.yaml`. The parser is a small project-local parser, not a general YAML parser. If `MapPoint` fields change, update both load and save code.

`MapPoint` fields include:

- `id`, `x`, `y`;
- `fast`, `constant_speed`;
- `segment_custom_speed`, `segment_constant_speed`, `segment_speed_level`;
- `segment_linear_x`, `segment_max_angular_speed`, `segment_k_alpha`, `segment_k_beta`;
- `task_type`;
- `event_label`.

Fast-marker validation is in `navigation_map_helpers.cpp`. Obstacle mode validates fast markers before saving or starting.

Waypoint `event_label` conventions:

- Legacy labels without a leading `@` keep the old behavior: `stand`, `bridge`, `low`, and `_name` policy labels.
- Labels with a leading `@` are special navigation labels and are drawn in green.
- `@stop_0.5` pauses at that point for 0.5 seconds; `@stop_1` pauses for 1 second.
- `@back_20` drives backward 20 cm after reaching the point. The number after `@back_` is centimeters.
- `@end_3` marks an end/start overlap point and prevents the previous 3 points from being selected
  as automatic startup points.

### UI

The UI is OpenCV HighGUI-rendered, not a native Qt widget UI.

Main files:

- `src/app/navigation_map_node.cpp`: node setup, timer loop, persistent UI state, remote subscriptions, window display.
- `src/app/navigation_ui_coordinator.cpp`: button/dropdown/text-input behavior, settings, mission planning, radar workflow.
- `src/app/navigation_mouse_controller.cpp`: mouse event routing, zoom/pan/click behavior.
- `src/app/navigation_points_workflow.cpp`: point file operations, add/remove/toggle markers, merge files, route patching, segment speed edits.
- `src/ui/map_ui_renderer.cpp`: panel/popup drawing and hit-testing.
- `include/ui/map_ui_types.hpp`: UI action and state structs.

The timer loop in `NavigationMapNode::onTimer()`:

1. installs scroll compatibility hooks;
2. checks remote heartbeat and operation timeouts if in remote mode;
3. obtains pose from remote state or local interface;
4. updates local controller in standalone mode;
5. builds `MapUiState`;
6. draws the map frame;
7. processes keyboard input;
8. periodically persists UI state to `config/ui_state.yaml`.

Do not assume UI state is disposable: `config/ui_state.yaml` is intentionally written by the app.

### Parameters

Parameter declaration and config persistence:

- `include/params/navigation_params.hpp`
- `src/params/navigation_params.cpp`

Runtime parameters include:

- `source`: `simulation` or `radar`;
- `robot_name`, `scene`, `points_file`;
- `show_window`, `cmd_vel_topic`;
- controller gains and speed limits;
- UI size/map dimensions;
- `race_logic`: `obstacle` or `mission`;
- mission/arm service settings;
- RL/debug event topics.

Controller parameter files live under `config/params/Sequential_Waypoint/`.

### Mission And Task Planning

Mission-specific code is split across:

- `src/app/navigation_runtime.cpp`: mission pause/resume, arm service protocol, task state.
- `src/app/navigation_ui_coordinator.cpp`: mission settings and route generation.
- `include/optim/task_order_planner.hpp`
- `src/optim/task_order_planner.cpp`
- `src/optim/task_order_planner.py`
- `config/mission/map_geom.yaml`

Mechanical arm protocol:

- Navigation sends arrived task requests to `arm_mission_service`, default `/arm/mission_event`.
- Arm or mock arm calls back into `navigation_arm_event_service`, default `/navigation/arm_event`.
- Events include `ack`, `grabbed`, `placed`, and `completed`.
- `scripts/mock_arm_node` can simulate this loop.

### Radar Calibration

Relevant files:

- `include/calibration/radar_calibration.hpp`
- `src/calibration/radar_calibration.cpp`
- `src/calibration/kabsch/kabsch.cpp`
- `src/interface/radar_pose_listener.cpp`
- `config/calibration/datas/*.yaml`
- `config/calibration/cali_params/*.yaml`

The UI can record radar/map point pairs, run registration, preview a transform, and accept it. In remote mode, accepting radar calibration sends `/navigation/set_radar_calibration` to core.

## ROS Interfaces

Generated interface files:

- `msg/MapPoint.msg`
- `srv/SetWaypoints.srv`
- `srv/SetControllerConfig.srv`
- `srv/StartNavigation.srv`
- `srv/StopNavigation.srv`
- `srv/StringCommand.srv`
- `srv/MissionCommand.srv`

There are duplicate arm-facing service definitions under `arm_integration/` for external integration/reference.

`StartNavigation` is intended to be atomic on the core side: stop old navigation, set points, apply config, validate markers, create controller, start.

## Launchers

Launch files:

- `launch/navigation.launch.py`: generic navigation node launch. It can launch different executables through `navigation_executable`.
- `launch/mujoco_navigation.launch.py`: starts MuJoCo runner, midware, and standalone navigation.

Tkinter scripts:

- `scripts/navigation_launcher`: general desktop launcher for simulation/radar and node roles.
- `scripts/navigation_core_launcher`: core-only launcher with radar/LIO mapping support.
- `scripts/navigation_ui_launcher`: remote UI-only launcher.
- `scripts/mock_arm_node`: mock mechanical arm service/event node for mission testing.

The launcher scripts contain environment-specific paths under the user's home directory, especially lidar/LIO workspaces.

## Data Layout

- `config/points/`: route point files.
- `config/params/Sequential_Waypoint/`: controller parameter presets.
- `config/maps/blackW/`: MJCF/XML/URDF map assets and meshes.
- `config/mission/`: mission geometry and preview tooling.
- `config/calibration/datas/`: sampled calibration point data.
- `config/calibration/cali_params/`: generated calibration transforms.
- `config/ui_state.yaml`: app-persisted UI state.
- `config/manual_tuning.yaml`: manual tuning data.

## Common Change Notes

- If you add or change `MapPoint` fields, update:
  - `msg/MapPoint.msg`;
  - `maps::MapPoint` in `top_view_map.hpp`;
  - point load/save in `point_store.cpp`;
  - UI rendering/editing if the field is user-facing;
  - remote service conversion code in `navigation_runtime.cpp` and `navigation_core_node.cpp`.
- If you add controller config fields, update:
  - `ControllerConfig`;
  - `makeControllerParamFields`;
  - service definitions for config/start;
  - request fill/apply functions in runtime/core;
  - UI parameter persistence.
- If you alter remote mode, keep service behavior, heartbeat, status parsing, and UI optimistic state in sync.
- If you alter mission behavior, inspect both runtime mission state and UI task-order generation.
- Avoid touching `config/ui_state.yaml` unless the task is explicitly about persisted UI state; it is often modified by running the UI.

## Current Worktree Note

At the time this file was created, the worktree already had modified files:

- `config/points/entir2.yaml`
- `config/ui_state.yaml`

Treat those as user/runtime changes unless the user explicitly asks to edit them.
