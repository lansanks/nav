# Navigation 跨机器通信：Topic → Service 改造

## 背景

Navigation 系统有机载节点（`navigation_core`）和远程 UI 节点（`navigation_remote_ui`）两个在不同机器上运行的节点。原先通过 ROS 2 Topic `/navigation/command` 传递控制指令，存在以下问题：

1. **Fire-and-forget**：UI 发完命令立即乐观更新本地状态，不等待确认
2. **非原子操作**：`start` 需要发 3 条独立消息（set_points + set_config + start），部分到达会导致不一致
3. **无错误反馈**：core 处理失败只记日志，UI 不知道
4. **无连接检测**：网络断开 UI 无法感知，永远显示最后状态

## 改造方案

将 UI → Core 的控制指令从 **Topic** 改为 **ROS 2 Service**，新增心跳机制。

### 通信架构对比

```
旧架构:
  Remote UI ──Topic──►  Core      (fire-and-forget, 无确认)
  Remote UI ◄──Topic──  Core      (state/status 持续推送)

新架构:
  Remote UI ──Service─►  Core      (请求-响应, 有确认)
  Remote UI ◄──Topic───  Core      (state/status/heartbeat 持续推送)
```

## 新增接口定义

### Message: `navigation/msg/MapPoint.msg`

```yaml
int32 id
float64 x
float64 y
bool fast
```

### Service: `navigation/srv/SetWaypoints.srv`

```yaml
MapPoint[] points
---
bool success
string message
```

用途：编辑路径点时同步到 core（导航未启动时使用）

### Service: `navigation/srv/SetControllerConfig.srv`

```yaml
float64 waypoint_tolerance
float64 max_linear_speed
float64 max_angular_speed
float64 k_rho
float64 k_alpha
float64 k_beta
float64 fast_max_linear_speed
float64 fast_max_angular_speed
float64 fast_k_rho
float64 fast_k_alpha
float64 fast_k_beta
---
bool success
string message
```

用途：在线调参时实时同步到 core

### Service: `navigation/srv/StartNavigation.srv`

```yaml
MapPoint[] points
float64 waypoint_tolerance
float64 max_linear_speed
float64 max_angular_speed
float64 k_rho
float64 k_alpha
float64 k_beta
float64 fast_max_linear_speed
float64 fast_max_angular_speed
float64 fast_k_rho
float64 fast_k_alpha
float64 fast_k_beta
string controller_name
---
bool success
string message
```

用途：**原子化**启动导航。一次调用完成：停止旧导航 → 设置路径点 → 设置参数 → 校验 fast markers → 启动控制器。任何一步失败返回 `success=false`。

### Service: `navigation/srv/StopNavigation.srv`

```yaml
string reason
---
bool success
string message
```

用途：停止导航

### Topic: `/navigation/heartbeat` (新增)

```
Type: std_msgs/String
Direction: Core → Remote UI
Rate: 1 Hz
Content: "beat"
```

Remote UI 如果超过 3 秒未收到心跳，判定 core 断开连接，面板显示 "Core: DISCONNECTED"。

## 修改的文件清单

### 新增 (5 个)

| 文件 | 说明 |
|------|------|
| `msg/MapPoint.msg` | 路径点消息 |
| `srv/SetWaypoints.srv` | 设置路径点 |
| `srv/SetControllerConfig.srv` | 设置控制器参数 |
| `srv/StartNavigation.srv` | 原子化启动 |
| `srv/StopNavigation.srv` | 停止导航 |

### 修改 (10 个)

| 文件 | 改动 |
|------|------|
| `CMakeLists.txt` | 添加 `rosidl_default_generators`、`rosidl_generate_interfaces`、链接 typesupport |
| `package.xml` | 添加 build/exec 依赖，加入 `rosidl_interface_packages` |
| `include/app/navigation_node_context.hpp` | 添加 4 个 service client 指针、心跳订阅、连接状态字段；移除 `remote_command_sender` function |
| `include/app/navigation_runtime.hpp` | 添加 4 个 private service 请求方法 |
| `include/app/navigation_map_node.hpp` | 移除 `publishRemoteCommand()`、`command_publisher_`；添加 `heartbeat_subscription_` |
| `include/ui/map_ui_types.hpp` | 添加 `core_connected` 字段 |
| `src/navigation_core_node.cpp` | 删除 command subscription + `handleCommand()`；新增 4 个 service server + heartbeat publisher |
| `src/app/navigation_runtime.cpp` | 删除 `sendRemoteCommand()`、序列化函数；替换为 4 个异步 service 调用方法 |
| `src/app/navigation_map_node.cpp` | 创建 service clients + heartbeat 订阅；`onTimer()` 中检测连接状态 |
| `src/app/navigation_ui_coordinator.cpp` | `buildUiState()` 传递 `core_connected` |
| `src/ui/map_ui_renderer.cpp` | 面板绘制 "Core: Connected / DISCONNECTED" |

## 关键实现细节

### 异步 Service 调用

所有 UI → Core 的 service 调用均使用 `async_send_request`，不阻塞 UI 线程：

```cpp
context_.start_client->async_send_request(
  request,
  [this](rclcpp::Client<StartNavigation>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      context_.remote_navigation_active = true;  // 确认后才更新
    } else {
      context_.remote_navigation_active = false;
      context_.status_message = "Core rejected: " + response->message;
    }
  });
```

### 原子启动

`StartNavigation` 的 core 端 handler 在一个 service callback 中顺序执行：

```
stop旧导航(如果需要) → setPoints → setConfig → validateFastMarkers → createController → controller.start()
```

任何一步失败立即返回 `success=false` + 错误原因，不会出现"部分生效"的状态。

### 连接检测

```
Core: 每秒发布 heartbeat → /navigation/heartbeat
UI:   订阅 heartbeat，更新 last_heartbeat_time
      onTimer() 中检查 last_heartbeat_time > 3s → core_connected = false
      同时检查 service 是否 ready → 双重判断
```

## 向后兼容

- `standalone` 模式（一体节点）行为完全不变
- 启动参数变化：
  - `navigation_command_topic` 参数在 remote_ui 模式下变为无效（无害保留）
  - service 名称固定在 `/navigation/{set_waypoints,set_config,start,stop}`
- **Remote UI 和 Core 必须同步升级**，旧版不兼容新版 service 接口
- Topic `/navigation/state` 和 `/navigation/status` 保持不变（Core → UI 方向）

## 构建

```bash
cd src
colcon build --packages-select navigation
```
