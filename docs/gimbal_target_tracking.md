# Gimbal Target Tracking

本文档说明如何让目标无人机尽量保持在主机 `x500_0` 云台相机画面中心。

## 设计结论

当前仓库已经具备三块基础能力：

1. 双机 PX4 SITL：主机使用 `/fmu/...`，目标机使用 `/px4_1/fmu/...`。
2. 主机云台相机图像：`/x500_0/camera/image_raw`。
3. YOLO 检测结果：`/x500_0/yolo/detections`，类型为 `vision_msgs/Detection2DArray`。

缺少的是一个视觉闭环节点：把检测框中心与图像中心的偏差转换为云台 yaw / pitch 指令。因此本分支新增：

```text
/x500_0/camera/image_raw
        │
        ▼
yolo_detector ──► /x500_0/yolo/detections
        │                     │
        │                     ▼
        │          gimbal_target_tracker
        │                     │
        │                     ▼
        └────────► /fmu/in/vehicle_command
```

`gimbal_target_tracker` 不控制无人机位置，只控制主机云台。无人机 0 的航点跟踪、目标机的航点跟踪、YOLO 检测仍然保持独立。

## 与开源 ROS 仓库的关系

公开 ROS/PX4/Gazebo 项目里常见的是以下几类：

- MAVROS / MAVLink mount control：提供通用云台命令接口，但通常不包含“检测框中心误差 → 云台视觉伺服”的完整应用闭环。
- YOLO ROS wrapper：负责图像检测，但一般只输出 bbox，不负责 PX4 gimbal manager 指令。
- 多无人机 Gazebo / PX4 demo：负责多机仿真或航点控制，但不直接解决“目标无人机始终位于某一架机云台相机中心”的问题。

因此，本仓库更合适的做法不是直接套一个外部仓库，而是在现有 ROS 2 / PX4 / Gazebo Harmonic 工程中增加一个轻量视觉伺服节点，并复用现有的 YOLO 输出和 PX4 `VehicleCommand` 通道。

## 启动方式

常规流程仍然是先启动 XRCE Agent、PX4/Gazebo、目标机和目标机航点控制：

```bash
./scripts/start_agent.sh
./scripts/start_px4_gazebo.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_waypoint_tracking.sh
```

然后启动主机航点跟踪、相机桥接、YOLO 检测和云台跟踪：

```bash
ENABLE_GIMBAL_TRACKING=true ./scripts/start_waypoint_tracking.sh
```

如果只想测试视觉闭环而不让主机移动，可以把主机航点 YAML 改成悬停点，或者单独启动节点：

```bash
ros2 run uav_waypoint_tracking gimbal_target_tracker \
  --ros-args \
  -p detections_topic:=/x500_0/yolo/detections \
  -p image_topic:=/x500_0/camera/image_raw \
  -p vehicle_command_topic:=/fmu/in/vehicle_command
```

## 常用参数

启动脚本暴露了最常用的环境变量：

```bash
ENABLE_GIMBAL_TRACKING=true \
GIMBAL_TARGET_CLASS_ID=uav \
GIMBAL_MIN_SCORE=0.35 \
GIMBAL_YAW_RATE_GAIN_DEG_S=45.0 \
GIMBAL_PITCH_RATE_GAIN_DEG_S=35.0 \
./scripts/start_waypoint_tracking.sh
```

含义如下：

- `ENABLE_GIMBAL_TRACKING`：是否启动视觉闭环云台跟踪。
- `GIMBAL_TARGET_CLASS_ID`：指定 YOLO 类别名或类别 id；为空时跟踪最高置信度目标。
- `GIMBAL_MIN_SCORE`：检测置信度阈值。
- `GIMBAL_YAW_RATE_GAIN_DEG_S`：水平方向图像误差到 yaw 角速度的比例增益。
- `GIMBAL_PITCH_RATE_GAIN_DEG_S`：垂直方向图像误差到 pitch 角速度的比例增益。
- `GIMBAL_YAW_ERROR_SIGN`：如果目标越跟越偏，改成 `-1.0`。
- `GIMBAL_PITCH_ERROR_SIGN`：如果目标越跟越偏，改成 `1.0`。

## 控制律

节点使用图像平面的归一化误差：

```text
error_x = (bbox_center_x - image_width  / 2) / (image_width  / 2)
error_y = (bbox_center_y - image_height / 2) / (image_height / 2)
```

然后进行死区处理、限幅和积分：

```text
yaw_rate   = yaw_error_sign   * yaw_gain   * error_x
pitch_rate = pitch_error_sign * pitch_gain * error_y

yaw_cmd   = clamp(yaw_cmd   + yaw_rate   * dt, min_yaw,   max_yaw)
pitch_cmd = clamp(pitch_cmd + pitch_rate * dt, min_pitch, max_pitch)
```

最后通过 `VehicleCommand` 发送 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`。默认角度单位为 degree。

## 验证话题

启动后检查以下话题：

```bash
ros2 topic list | rg 'x500_0/(camera|yolo|gimbal)'
ros2 topic echo /x500_0/yolo/detections --once
ros2 topic echo /x500_0/gimbal_target_tracker/error --once
ros2 topic echo /x500_0/gimbal_target_tracker/tracking_active --once
```

如果 `tracking_active` 为 `false`，优先检查：

1. `/x500_0/camera/image_raw` 是否有图像。
2. `/x500_0/yolo/detections` 是否有检测框。
3. `GIMBAL_TARGET_CLASS_ID` 是否和 YOLO 输出的 `class_id` 完全一致。
4. `GIMBAL_MIN_SCORE` 是否过高。

## 调参建议

1. 先不开主机航点，只让目标机运动，确认云台能把目标拉回中心。
2. 如果目标向右偏，云台也继续向右导致更偏，反转 `GIMBAL_YAW_ERROR_SIGN`。
3. 如果目标向上偏，云台也继续向上导致更偏，反转 `GIMBAL_PITCH_ERROR_SIGN`。
4. 目标在画面中振荡时，降低 `GIMBAL_YAW_RATE_GAIN_DEG_S` 和 `GIMBAL_PITCH_RATE_GAIN_DEG_S`。
5. 目标移动快但云台跟不上时，适当提高增益和最大角速度。

## 局限性

当前实现是基于图像误差的二维视觉伺服，不估计目标三维位置，也不预测目标运动。它适合先验证“目标是否能被持续保持在画面中心”。如果后续要做更强的导引头/制导仿真，可以进一步增加：

- bbox 跟踪器，例如 SORT / ByteTrack，减少 YOLO 间歇丢检带来的抖动；
- 目标相对方位角、俯仰角估计；
- 基于目标速度的前馈补偿；
- 与无人机 0 航迹规划联动，使机体位置和云台角度共同保持目标可见。
