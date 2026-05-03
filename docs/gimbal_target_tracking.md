# Gimbal Target Tracking

本文档说明如何让目标无人机尽量保持在主机 `x500_0` 的云台相机画面中心。

## 设计结论

当前仓库的视觉闭环主链路已经收敛为 **YOLO + ByteTrack 跟踪**，而不是逐帧检测：

1. 双机 PX4 SITL：主机使用 `/fmu/...`，目标机使用 `/px4_1/fmu/...`。
2. 主机云台相机图像：`/x500_0/camera/image_raw`。
3. YOLO + ByteTrack 跟踪结果：`/x500_0/yolo/tracks`，类型为 `vision_msgs/Detection2DArray`。
4. `Detection2D.id` 表示跨帧持续的 `track_id`，不是单帧检测序号。
5. 云台节点默认锁定同一个 `track_id`；目标短时丢失时保持原云台指令，丢失超过 `lost_timeout_s` 后释放锁定并允许重新捕获。

数据流如下：

```text
/x500_0/camera/image_raw
        │
        ▼
yolo_tracker(ByteTrack) ──► /x500_0/yolo/tracks
        │                              │
        │                              ▼
        │                 gimbal_target_tracker
        │                    ▲         │
        │                    │         │
        │     /fmu/out/gimbal_device_attitude_status
        │                              │
        │                              ▼
        ├────────────────► /fmu/in/gimbal_manager_set_attitude
        └──── configure ─► /fmu/in/vehicle_command
```

`gimbal_target_tracker` 不控制无人机位置，只控制主机云台。无人机 0 的航点跟踪、目标机的航点跟踪、ByteTrack 目标跟踪和云台视觉伺服保持分层独立。

## 为什么只保留 tracking

逐帧检测只能得到当前帧 bbox，无法判断相邻帧中的目标是否为同一个实体。对于“目标无人机始终位于云台相机中心”这个任务，云台控制更需要稳定的跨帧目标中心。因此当前项目只保留 `yolo_tracker` 作为视觉节点：

- 避免 `yolo_detector` 与 `yolo_tracker` 同时运行造成重复 YOLO 推理；
- 使用 `/x500_0/yolo/tracks` 作为唯一视觉输出，接口更清晰；
- 保持 `vision_msgs/Detection2DArray` 消息类型不变，降低下游节点耦合；
- 通过 `Detection2D.id` 保留跨帧 track id，云台节点可以锁定同一目标，避免多目标场景中频繁切换。

## 与开源 ROS 仓库的关系

公开 ROS/PX4/Gazebo 项目里常见的是以下几类：

- MAVROS / MAVLink mount control：提供通用云台命令接口，但通常不包含“跟踪目标中心误差 → 云台视觉伺服”的完整应用闭环。
- YOLO ROS wrapper：负责图像检测或跟踪，但一般不直接负责 PX4 gimbal manager 指令。
- 多无人机 Gazebo / PX4 demo：负责多机仿真或航点控制，但不直接解决“目标无人机始终位于某一架机云台相机中心”的问题。

因此，本仓库采用轻量集成方案：在现有 ROS 2 / PX4 / Gazebo Harmonic 工程中加入 `yolo_tracker` 和 `gimbal_target_tracker` 两个独立节点。云台连续闭环使用 PX4 gimbal manager 高频 setpoint topic，`VehicleCommand` 只用于一次性 gimbal manager 配置和兼容回退。

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
  -p camera_info_topic:=/x500_0/camera/camera_info \
  -p gimbal_attitude_topic:=/fmu/out/gimbal_device_attitude_status \
  -p gimbal_set_attitude_topic:=/fmu/in/gimbal_manager_set_attitude \
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
- `target_track_id`
- `lock_target_track`
- `min_score`
- `lost_timeout_s`
- `fallback_fx_px`
- `fallback_fy_px`
- `fallback_cx_px`
- `fallback_cy_px`
- `deadband_angle_deg`
- `yaw_rate_gain_s_inv`
- `pitch_rate_gain_s_inv`
- `yaw_error_sign`
- `pitch_error_sign`
- `yaw_frame`
- `command_interface`
- `hold_last_command_on_loss`
- `use_gimbal_feedback`
- `configure_gimbal_manager`
- `configure_retry_period_s`
- `configure_max_attempts`

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
- `GIMBAL_ATTITUDE_TOPIC`：云台真实姿态反馈 topic，默认 `/fmu/out/gimbal_device_attitude_status`。
- `GIMBAL_SET_ATTITUDE_TOPIC`：云台高频 setpoint topic，默认 `/fmu/in/gimbal_manager_set_attitude`。
- `CAMERA_INFO_TOPIC`：云台节点订阅的 ROS `sensor_msgs/CameraInfo` topic，默认 `/x500_0/camera/camera_info`。
- `YOLO_TRACKING_CONFIG_FILE`：可选，自定义 YOLO/ByteTrack 参数文件。
- `GIMBAL_CONFIG_FILE`：可选，自定义云台控制参数文件。

## 控制律

云台节点优先订阅 `sensor_msgs/CameraInfo`，使用相机内参矩阵 `K` 中的独立焦距和主点，把跟踪框中心的像素误差转换成视线角误差：

```text
fx = K[0]
fy = K[4]
cx = K[2]
cy = K[5]

yaw_error_deg   = degrees(atan2(track_center_x - cx, fx))
pitch_error_deg = degrees(atan2(track_center_y - cy, fy))
```

如果 `CameraInfo` 暂时未到达，节点使用 YAML 中的显式 fallback 内参；fallback 是启动容错，不是主路径。

然后进行死区处理、限幅和积分：

```text
yaw_rate   = yaw_error_sign   * yaw_rate_gain_s_inv   * yaw_error_deg
pitch_rate = pitch_error_sign * pitch_rate_gain_s_inv * pitch_error_deg

yaw_cmd   = clamp(yaw_cmd   + yaw_rate   * dt, min_yaw,   max_yaw)
pitch_cmd = clamp(pitch_cmd + pitch_rate * dt, min_pitch, max_pitch)
```

最后默认通过 `/fmu/in/gimbal_manager_set_attitude` 发布 `px4_msgs/msg/GimbalManagerSetAttitude`。这对应 MAVLink gimbal manager 的高频 setpoint 路径：MAVLink 标准中 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` 是低频命令，连续流式控制应使用 `GIMBAL_MANAGER_SET_PITCHYAW`；PX4 在 ROS 2 / uORB 侧用 `gimbal_manager_set_attitude` 承载等价的姿态 setpoint。节点会把内部的 `pitch/yaw` 角转换成四元数 `q` 发布，角速度字段设为 `NaN`，表示只给角度目标。

节点仍会通过 `/fmu/in/vehicle_command` 发送 `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`，把当前 `source_system/source_component` 设置为 gimbal manager primary control。节点会订阅 `/fmu/out/vehicle_command_ack`，在收到 accepted ACK 前按 `configure_retry_period_s` 周期重试，避免 PX4/gimbal 尚未准备好时一次性配置丢失。如需回退到旧实现，可将 `command_interface` 改为 `vehicle_command`，此时会连续发送 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`。

诊断话题 `/x500_0/gimbal_target_tracker/error` 中的 `vector.x/y` 现在分别表示 yaw/pitch 视线角误差，单位为 degree，`vector.z` 为目标置信度。

`yaw_frame` 默认是 `vehicle`，节点在 PX4 ROS 2 / uORB 路径上发送 `flags=0`，使 yaw 按机体系解释；如需地理系 yaw，可改为 `earth`，节点会发送 `GIMBAL_MANAGER_FLAGS_YAW_LOCK`。当前 PX4 的 `gimbal_manager_set_attitude` 处理逻辑实际只用 `YAW_LOCK` 区分 yaw body frame 与 earth frame。

节点默认订阅 `/fmu/out/gimbal_device_attitude_status`，并在反馈消息的 yaw frame 与当前 `yaw_frame` 配置一致时，用云台反馈四元数同步内部 `pitch/yaw` 状态；这样控制积分从实际云台角度继续，而不是只依赖上一次发送的命令。当前 Gazebo 云台反馈按 vehicle frame 发布，因此默认 `yaw_frame: vehicle` 与反馈一致。如果需要回退到纯命令积分，可将 `use_gimbal_feedback` 设为 `false`。

注意：本机 PX4 的 `/home/zk/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 已加入 `/fmu/in/gimbal_manager_set_attitude`。修改该文件后需要重新启动 `scripts/start_px4_gazebo.sh`，让 PX4 重新构建并生成 XRCE-DDS topic 支持。

## 目标锁定

`lock_target_track` 控制是否锁定同一个 `Detection2D.id`。当前 YAML 配置为 `true`，节点会在通过 `target_class_id`、`min_score` 筛选后锁定一个 `track_id`，降低多目标场景中的切换概率；如果 ByteTrack 临时丢失并重新分配 ID，节点会优先重锁到离上一次目标图像位置最近的候选。如果改为 `false`，则每帧跟踪筛选后的最高置信度目标：

```text
target_track_id 非空：只跟踪指定 track id
target_track_id 为空且 lock_target_track=true：自动锁定首次选中的 track id
锁定目标丢失超过 lost_timeout_s：释放自动锁定，允许重新捕获
```

如果只需要每帧跟踪最高置信度目标，可把 `lock_target_track` 设为 `false`。

## 验证话题

每个新终端执行 ROS 2 命令前都需要先加载本工作区环境，否则 `px4_msgs` 自定义消息无法解析，`ros2 topic echo /fmu/out/gimbal_device_attitude_status --once` 会报 `The message type 'px4_msgs/msg/GimbalDeviceAttitudeStatus' is invalid`：

```bash
cd /home/zk/uav_waypoint_tracking_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果刚刚切换过环境或重编译过 `px4_msgs`，先重启 ROS daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

启动后检查以下话题：

```bash
ros2 topic list | rg 'x500_0/(camera|yolo|gimbal)'
ros2 topic echo /x500_0/camera/camera_info --once
ros2 topic echo /x500_0/yolo/tracks --once
ros2 topic echo /fmu/in/gimbal_manager_set_attitude --once
ros2 topic echo /fmu/out/gimbal_device_attitude_status --once
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
3. `gimbal_tracking.yaml` 中的 `target_track_id` 是否指定了当前不存在的 track id。
4. `gimbal_tracking.yaml` 中的 `min_score` 是否过高。
5. `/fmu/out/gimbal_device_attitude_status` 是否有云台姿态反馈。
6. `/fmu/in/gimbal_manager_set_attitude` 是否持续有 `GimbalManagerSetAttitude`。
7. `GIMBAL_INPUT_TOPIC` 是否与 `YOLO_TRACKS_TOPIC` 一致。

如果 `ros2 topic echo /fmu/out/gimbal_device_attitude_status --once` 提示消息类型无效，优先检查当前终端是否已经 `source install/setup.bash`，并确认 `ros2 interface show px4_msgs/msg/GimbalDeviceAttitudeStatus` 能正常显示字段。

## 调参建议

1. 先不开主机航点，只让目标机运动，确认 `/x500_0/yolo/tracks` 中的 `Detection2D.id` 能保持相对稳定。
2. 再打开 `ENABLE_GIMBAL_TRACKING=true`，观察云台是否能把目标拉回画面中心。
3. 如果目标向右偏，云台也继续向右导致更偏，反转 `gimbal_tracking.yaml` 中的 `yaw_error_sign`。
4. 如果目标向上偏，云台也继续向上导致更偏，反转 `pitch_error_sign`。
5. 如果画面里有两个以上目标且跟踪目标来回切换，优先指定 `target_track_id`，或提高 YOLO/ByteTrack 的跟踪稳定性。
6. 目标在画面中振荡时，降低 `yaw_rate_gain_s_inv` 和 `pitch_rate_gain_s_inv`。
7. 目标移动快但云台跟不上时，适当提高增益和最大角速度。

## 局限性

当前实现是基于图像误差的二维视觉伺服，不估计目标三维位置，也不预测目标运动。ByteTrack 可以提升跨帧目标连续性，但它仍依赖 YOLO 检测结果；当目标长时间遮挡、过小或置信度过低时，track id 可能丢失，并在释放锁定后重新捕获。

如果后续要做更强的导引头/制导仿真，可以进一步增加：

- 基于目标速度的前馈补偿；
- 目标相对方位角、俯仰角估计；
- 与无人机 0 航迹规划联动，使机体位置和云台角度共同保持目标可见。
