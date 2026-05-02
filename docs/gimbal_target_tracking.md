# Gimbal Target Tracking

本文档说明如何让目标无人机尽量保持在主机 `x500_0` 的云台相机画面中心。

## 设计结论

当前仓库的视觉闭环主链路已经收敛为 **YOLO + ByteTrack 跟踪**，而不是逐帧检测：

1. 双机 PX4 SITL：主机使用 `/fmu/...`，目标机使用 `/px4_1/fmu/...`。
2. 主机云台相机图像：`/x500_0/camera/image_raw`。
3. YOLO + ByteTrack 跟踪结果：`/x500_0/yolo/tracks`，类型为 `vision_msgs/Detection2DArray`。
4. `Detection2D.id` 表示跨帧持续的 `track_id`，不是单帧检测序号。

数据流如下：

```text
/x500_0/camera/image_raw
        │
        ▼
yolo_tracker(ByteTrack) ──► /x500_0/yolo/tracks
        │                              │
        │                              ▼
        │                 gimbal_target_tracker
        │                              │
        │                              ▼
        └────────────────► /fmu/in/vehicle_command
```

`gimbal_target_tracker` 不控制无人机位置，只控制主机云台。无人机 0 的航点跟踪、目标机的航点跟踪、ByteTrack 目标跟踪和云台视觉伺服保持分层独立。

## 为什么只保留 tracking

逐帧检测只能得到当前帧 bbox，无法判断相邻帧中的目标是否为同一个实体。对于“目标无人机始终位于云台相机中心”这个任务，云台控制更需要稳定的跨帧目标中心。因此当前项目只保留 `yolo_tracker` 作为视觉节点：

- 避免 `yolo_detector` 与 `yolo_tracker` 同时运行造成重复 YOLO 推理；
- 使用 `/x500_0/yolo/tracks` 作为唯一视觉输出，接口更清晰；
- 保持 `vision_msgs/Detection2DArray` 消息类型不变，降低下游节点耦合；
- 通过 `Detection2D.id` 保留跨帧 track id，方便后续扩展“锁定同一目标”。

## 与开源 ROS 仓库的关系

公开 ROS/PX4/Gazebo 项目里常见的是以下几类：

- MAVROS / MAVLink mount control：提供通用云台命令接口，但通常不包含“跟踪目标中心误差 → 云台视觉伺服”的完整应用闭环。
- YOLO ROS wrapper：负责图像检测或跟踪，但一般不直接负责 PX4 gimbal manager 指令。
- 多无人机 Gazebo / PX4 demo：负责多机仿真或航点控制，但不直接解决“目标无人机始终位于某一架机云台相机中心”的问题。

因此，本仓库采用轻量集成方案：在现有 ROS 2 / PX4 / Gazebo Harmonic 工程中加入 `yolo_tracker` 和 `gimbal_target_tracker` 两个独立节点，并复用 PX4 `VehicleCommand` 通道。

## 启动方式

常规流程仍然是先启动 XRCE Agent、PX4/Gazebo、目标机和目标机航点控制：

```bash
./scripts/start_agent.sh
./scripts/start_px4_gazebo.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_waypoint_tracking.sh
```

然后启动主机航点跟踪、相机桥接、ByteTrack 跟踪和云台视觉伺服：

```bash
ENABLE_GIMBAL_TRACKING=true ./scripts/start_waypoint_tracking.sh
```

`start_waypoint_tracking.sh` 默认启用 `ENABLE_YOLO_TRACKING=true`，因此一般不需要额外指定跟踪开关。

如果只想测试视觉链路，可以先不开云台闭环：

```bash
ENABLE_GIMBAL_TRACKING=false ./scripts/start_waypoint_tracking.sh
```

如果只想测试云台闭环而不让主机移动，可以把主机航点 YAML 改成悬停点，或者单独启动云台节点并订阅跟踪结果：

```bash
ros2 run uav_waypoint_tracking gimbal_target_tracker \
  --ros-args \
  -p detections_topic:=/x500_0/yolo/tracks \
  -p image_topic:=/x500_0/camera/image_raw \
  -p vehicle_command_topic:=/fmu/in/vehicle_command
```

## 配置文件

默认参数主要来自两个 YAML 文件：

```text
src/uav_waypoint_tracking/config/yolo_tracking.yaml
src/uav_waypoint_tracking/config/gimbal_tracking.yaml
```

`yolo_tracking.yaml` 管理 YOLO/ByteTrack 参数，例如：

- `tracker_config: bytetrack.yaml`
- `confidence_threshold`
- `iou_threshold`
- `image_size`
- `max_detections`
- `classes`
- `device`

`gimbal_tracking.yaml` 管理云台视觉伺服参数，例如：

- `target_class_id`
- `min_score`
- `image_width`
- `image_height`
- `horizontal_fov_rad`
- `deadband_angle_deg`
- `yaw_rate_gain_s_inv`
- `pitch_rate_gain_s_inv`
- `yaw_error_sign`
- `pitch_error_sign`
- `hold_last_command_on_loss`

配置优先级为：

```text
命令行 launch 参数 / 启动脚本环境变量 > YAML 配置文件 > 节点内置默认值
```

## 常用环境变量

启动脚本只暴露运行入口和 topic 相关参数，算法默认值建议优先写入 YAML：

```bash
ENABLE_YOLO_TRACKING=true \
ENABLE_GIMBAL_TRACKING=true \
YOLO_TRACKS_TOPIC=/x500_0/yolo/tracks \
GIMBAL_INPUT_TOPIC=/x500_0/yolo/tracks \
./scripts/start_waypoint_tracking.sh
```

含义如下：

- `ENABLE_YOLO_TRACKING`：是否启动 YOLO + ByteTrack 跟踪，默认 `true`。
- `ENABLE_GIMBAL_TRACKING`：是否启动视觉闭环云台跟踪，`start_waypoint_tracking.sh` 默认 `true`。
- `YOLO_TRACKS_TOPIC`：`yolo_tracker` 发布的跟踪结果 topic。
- `GIMBAL_INPUT_TOPIC`：`gimbal_target_tracker` 订阅的 `Detection2DArray` topic，默认等于 `YOLO_TRACKS_TOPIC`。
- `YOLO_TRACKING_CONFIG_FILE`：可选，自定义 YOLO/ByteTrack 参数文件。
- `GIMBAL_CONFIG_FILE`：可选，自定义云台控制参数文件。

## 控制律

云台节点使用云台相机的针孔模型，把跟踪框中心的像素误差转换成视线角误差。当前 Gazebo 相机参数为 `1280x720`，水平视场角 `horizontal_fov_rad=2.0`：

```text
fx = image_width / (2 * tan(horizontal_fov_rad / 2))

yaw_error_deg   = degrees(atan2(track_center_x - image_width  / 2, fx))
pitch_error_deg = degrees(atan2(track_center_y - image_height / 2, fx))
```

然后进行死区处理、限幅和积分：

```text
yaw_rate   = yaw_error_sign   * yaw_rate_gain_s_inv   * yaw_error_deg
pitch_rate = pitch_error_sign * pitch_rate_gain_s_inv * pitch_error_deg

yaw_cmd   = clamp(yaw_cmd   + yaw_rate   * dt, min_yaw,   max_yaw)
pitch_cmd = clamp(pitch_cmd + pitch_rate * dt, min_pitch, max_pitch)
```

最后通过 `VehicleCommand` 发送 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`。默认角度单位为 degree。诊断话题 `/x500_0/gimbal_target_tracker/error` 中的 `vector.x/y` 现在分别表示 yaw/pitch 视线角误差，单位为 degree，`vector.z` 为目标置信度。

## 验证话题

启动后检查以下话题：

```bash
ros2 topic list | rg 'x500_0/(camera|yolo|gimbal)'
ros2 topic echo /x500_0/yolo/tracks --once
ros2 topic echo /x500_0/gimbal_target_tracker/error --once
ros2 topic echo /x500_0/gimbal_target_tracker/tracking_active --once
```

如果 `/x500_0/yolo/tracks` 没有输出，优先检查：

1. `/x500_0/camera/image_raw` 是否有图像。
2. YOLO 权重文件是否存在。
3. `yolo_tracking.yaml` 中的 `classes` 是否过滤掉了目标类别。
4. `confidence_threshold` 是否过高。

如果 `tracking_active` 为 `false`，优先检查：

1. `/x500_0/yolo/tracks` 是否持续有跟踪框。
2. `gimbal_tracking.yaml` 中的 `target_class_id` 是否和 YOLO 输出的 `class_id` 完全一致。
3. `gimbal_tracking.yaml` 中的 `min_score` 是否过高。
4. `GIMBAL_INPUT_TOPIC` 是否与 `YOLO_TRACKS_TOPIC` 一致。

## 调参建议

1. 先不开主机航点，只让目标机运动，确认 `/x500_0/yolo/tracks` 中的 `Detection2D.id` 能保持相对稳定。
2. 再打开 `ENABLE_GIMBAL_TRACKING=true`，观察云台是否能把目标拉回画面中心。
3. 如果目标向右偏，云台也继续向右导致更偏，反转 `gimbal_tracking.yaml` 中的 `yaw_error_sign`。
4. 如果目标向上偏，云台也继续向上导致更偏，反转 `pitch_error_sign`。
5. 目标在画面中振荡时，降低 `yaw_rate_gain_s_inv` 和 `pitch_rate_gain_s_inv`。
6. 目标移动快但云台跟不上时，适当提高增益和最大角速度。

## 局限性

当前实现是基于图像误差的二维视觉伺服，不估计目标三维位置，也不预测目标运动。ByteTrack 可以提升跨帧目标连续性，但它仍依赖 YOLO 检测结果；当目标长时间遮挡、过小或置信度过低时，track id 可能丢失或切换。

如果后续要做更强的导引头/制导仿真，可以进一步增加：

- 基于 `Detection2D.id` 的目标锁定策略，避免多目标场景中频繁切换目标；
- 目标相对方位角、俯仰角估计；
- 基于目标速度的前馈补偿；
- 与无人机 0 航迹规划联动，使机体位置和云台角度共同保持目标可见。
