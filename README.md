# UAV Trajectory Tracking Simulation

PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy 的本地 NED 参数化轨迹跟踪仿真工作区。当前视觉链路采用 **YOLO + BoT-SORT 目标跟踪**，云台节点负责把目标稳定到主机 `x500_0` 的相机中心；视觉拦截链路进一步使用 Gazebo truth 相对位置/速度做 LOS-rate 比例导引，云台锁定只作为导引头可用性门控。

## 环境

- PX4: `/home/zk/PX4-Autopilot`
- Python venv: `/home/zk/px4-venv`
- ROS 2: Jazzy
- Gazebo: Harmonic
- DDS Agent: `MicroXRCEAgent udp4 -p 8888`

## 构建

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/build.sh
```

## 启动

终端 1:

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_agent.sh
```

终端 2:

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_px4_gazebo.sh
```

如果只做主机轨迹跟踪，终端 3 启动主机轨迹跟踪、相机桥接、YOLO + BoT-SORT 跟踪、日志记录和 RViz 可视化发布节点：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_trajectory_tracking.sh
```

如果做视觉拦截，不要同时启动 `start_trajectory_tracking.sh` 控制主机，因为它和 `visual_pursuit_interceptor` 都会向 `/fmu/in/trajectory_setpoint` 发布 setpoint。视觉拦截流程从目标机开始：

终端 3，启动第二架目标无人机的 PX4 实例：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_target_px4_gazebo.sh
```

终端 4，让目标无人机按目标轨迹飞行：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_target_trajectory_tracking.sh
```

终端 5，启动主机视觉拦截链路。该入口会启动相机桥接、YOLO + BoT-SORT、云台视觉伺服、Gazebo truth odometry bridge 和 `visual_pursuit_interceptor`：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_visual_interception.sh
```

终端 6 可选，用于查看轨迹、目标点、规划路径和实际飞行尾迹：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_rviz.sh
```

当前 PX4 main 在这台机器上发布的是 `/fmu/out/vehicle_status_v4` 和
`/fmu/out/vehicle_local_position_v1`，启动文件默认已经使用这两个话题。
`start_trajectory_tracking.sh` 会同时启动控制节点、RViz 可视化发布节点、相机桥接和 YOLO + BoT-SORT 跟踪节点。

## 多机与坐标系

第二架目标无人机按 PX4 Gazebo 官方多机方式运行：每架机一个独立 PX4 SITL 实例。
主机使用默认实例 `px4_instance=0`、Gazebo 模型 `x500_0`、PX4 官方
`gz_x500_gimbal` airframe、ROS 2 话题 `/fmu/...`、`MAV_SYS_ID=1`。目标机使用
`scripts/start_target_px4_gazebo.sh` 启动为
`px4_instance=1`，连接 Gazebo 中预加载的 `x500_1`；PX4 会自动设置
`MAV_SYS_ID=2`、`UXRCE_DDS_KEY=2`，ROS 2 话题带 `/px4_1/...` namespace。
因此目标机的 Offboard 控制必须向 `/px4_1/fmu/in/...` 发布，并把
`VehicleCommand.target_system` 设为 `2`。`scripts/start_target_trajectory_tracking.sh`
已经按这个规则配置。`px4_instance > 0` 时不要复用主机的 `/fmu/in/*` 话题。

主机启动脚本默认轨迹在 `src/uav_trajectory_tracking/config/trajectory_hold.yaml`，目标机默认轨迹在
`src/uav_trajectory_tracking/config/target_trajectory_linear.yaml`。两者坐标系都是 PX4 本地 NED：

- `x`: 北/前
- `y`: 东/右
- `z`: 下，起飞点上方 5 m 写作 `-5.0`

只改主机默认轨迹时修改 `trajectory_hold.yaml` 或用 `TRAJECTORY_FILE=...` 覆盖；只改目标机轨迹时修改 `target_trajectory_linear.yaml` 或用 `TRAJECTORY_FILE=...` 覆盖。
`scripts/start_px4_gazebo.sh` 会在启动前自动根据主机 YAML
生成 Gazebo world，并同步到 PX4 的 worlds 目录；`scripts/start_trajectory_tracking.sh`
也会默认把同一个主机 YAML 传给控制节点和 RViz 可视化节点。目标机启动脚本默认使用
`target_trajectory_linear.yaml`，用于给拦截链路提供持续横向运动目标。若需要定点目标，可把
`TRAJECTORY_FILE` 指向 `src/uav_trajectory_tracking/config/target_trajectory_hold.yaml`。
轨迹控制采用 `entry -> trajectory -> return -> finished` 阶段：无人机先飞到曲线起点，
满足统一的 `acceptance_radius_m` 到达判定并短暂稳定后，才开始参数化时间 `t`。默认主机
`trajectory_hold.yaml` 的悬停点为 `(0, 0, -5)`；如果改用 `trajectory_figure8.yaml`，
8 字轨迹的交叉点、起点和终点均为 `(0, 0, -5)`，QGC 的水平轨迹不会包含额外的长距离进场线。

使用自定义轨迹文件时，PX4/Gazebo 终端和 ROS 终端都传入同一个变量：

```bash
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_px4_gazebo.sh
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_trajectory_tracking.sh
TRAJECTORY_FILE=/home/zk/my_target_trajectory.yaml ./scripts/start_target_trajectory_tracking.sh
```

Gazebo 如果暂停，点击左下角播放按钮。轨迹仿真默认使用
`trajectory_tracking` 世界，预加载主机 `x500_0` 和目标机 `x500_1`。`x500_0` 使用本仓库的
`x500_gimbal_trajectory_wind` 包装模型，内部使用带自机相机消隐的 `x500_gimbal_self_filtered`；
`x500_1` 仍使用普通
`x500_trajectory_wind`。两个机体都由 world 预加载，PX4 启动后分别通过
`PX4_GZ_MODEL_NAME=x500_0` 和 `PX4_GZ_MODEL_NAME=x500_1` 连接，因此可以只在本仓库内启用
受风模型、真值 odometry 和云台相机自机消隐，不修改 PX4 原始 `x500_base` / `x500_gimbal`。
`default1` 保留给 `/home/zk/gimbal_track` 使用，其中仍包含 `x500_target_moving`。
Gazebo 世界默认不插入轨迹线、边界、参照方块或风向箭头，以减少视觉识别干扰；RViz 使用 ROS ENU
`map` 坐标显示轨迹，其中 PX4 NED 会自动转换为 `x=east, y=north, z=up`。如果需要临时查看 Gazebo 静态轨迹标记，可用 `SHOW_TRAJECTORY_VISUALS=true ./scripts/start_px4_gazebo.sh` 启动。
Gazebo 世界坐标同样按 ENU 显示：`Gazebo x=east`、`Gazebo y=north`、`Gazebo z=up`。

## 云台相机、YOLO + BoT-SORT 跟踪与云台闭环

主机 `x500_0` 的云台相机图像按 ROS/Gazebo 官方 image bridge 接入 ROS 2：

- Gazebo 原始话题：`/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/image`
- ROS 2 图像话题：`/x500_0/camera/image_raw`
- YOLO + BoT-SORT 跟踪结果：`/x500_0/yolo/tracks`
- 跟踪标注图像：`/x500_0/yolo/tracks_image`

视觉主链路为：

```text
/x500_0/camera/image_raw
        │
        ▼
yolo_tracker(BoT-SORT) ──► /x500_0/yolo/tracks
        │                              │
        │                              ▼
        │                 gimbal_target_tracker
        │                    ▲         │
        │                    │         │
        │     /x500_0/gimbal/joint_states
        │                              │
        │                              ▼
        ├────────────────► /fmu/in/gimbal_manager_set_attitude
        └──── configure ─► /fmu/in/vehicle_command
```

`scripts/start_trajectory_tracking.sh` 默认启动相机桥接和 YOLO + BoT-SORT 跟踪，权重文件为仓库根目录
`yolov8s.pt`。YOLO Python 依赖当前安装在 `/home/zk/px4-venv`，`scripts/build.sh` 会使用这个
venv 构建 ROS 2 console scripts。

只需要轨迹控制、不需要视觉跟踪时：

```bash
ENABLE_YOLO_TRACKING=false ./scripts/start_trajectory_tracking.sh
```

只保留控制和日志，连相机图像桥接也关闭：

```bash
ENABLE_CAMERA_BRIDGE=false ENABLE_YOLO_TRACKING=false ./scripts/start_trajectory_tracking.sh
```

启动云台目标居中闭环：

```bash
ENABLE_GIMBAL_TRACKING=true ./scripts/start_trajectory_tracking.sh
```

默认情况下，`gimbal_target_tracker` 订阅 `GIMBAL_INPUT_TOPIC=/x500_0/yolo/tracks`、`/x500_0/camera/camera_info` 和 `/x500_0/gimbal/joint_states`，并根据跟踪框中心与相机内参计算出的视线角误差，向 `/fmu/in/gimbal_manager_set_attitude` 发布 PX4 gimbal manager 高频姿态 setpoint；`/x500_0/gimbal/joint_states` 是 Gazebo 云台关节反馈。`/fmu/in/vehicle_command` 用于 gimbal manager 配置和兼容回退，配置命令会重试直到 PX4 ACK。详细说明见 `docs/gimbal_target_tracking.md`。

云台节点同时发布两类状态：`tracking_active` 表示有新鲜目标检测/跟踪，`lock_active` 表示目标已经居中且残差稳定，外层导引可以使用。`lock_active` 由 yaw/pitch 图像误差、残差角速度、云台滞后状态和进入/退出滞回共同决定。

视觉拦截节点 `visual_pursuit_interceptor` 只在 `lock_active` 新鲜时进入 `pursuit`。当前仿真导引不再使用云台光轴作为相对几何，而是订阅 Gazebo truth odometry，计算真实相对位置、相对速度、LOS、LOS rate 和闭合速度，然后发布 PX4 NED velocity setpoint。云台光轴仍会显式计算并用于诊断 `gimbal_truth_los_error_deg`，用于判断云台导引头是否真的对准目标。详细说明见 `docs/visual_pursuit_interception.md`。

查看带 BoT-SORT 跟踪标签的相机窗口时，`rqt_image_view` 终端不要激活 `/home/zk/px4-venv`，
否则可能找不到系统 PyQt5：

```bash
deactivate
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_trajectory_tracking_sim/install/setup.bash
ros2 run rqt_image_view rqt_image_view /x500_0/yolo/tracks_image
```

手动执行 `ros2 topic echo`、`ros2 interface show` 等检查命令时，每个新终端都要先加载本工作区：

```bash
cd /home/zk/uav_trajectory_tracking_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果 PX4 相关 topic 提示 `px4_msgs` 消息类型无效，通常就是当前终端没有加载本工作区的 `px4_msgs`，或者 ROS daemon 还缓存着旧环境；执行 `ros2 daemon stop && ros2 daemon start` 后再试。

视觉链路常用配置文件：

- `src/uav_trajectory_tracking/config/yolo_tracking.yaml`: YOLO/BoT-SORT 参数，例如 `tracker_config`、`confidence_threshold`、`iou_threshold`、`image_size`、`max_inference_hz`、`classes`、`device`。
- `src/uav_trajectory_tracking/config/gimbal_tracking.yaml`: 云台视觉伺服参数，例如 `target_class_id`、`target_track_id`、`lock_target_track`、`min_score`、`fallback_fx_px`、`fallback_fy_px`、`deadband_angle_deg`、`yaw_kp_s_inv`、`pitch_kp_s_inv`、`lock_yaw_error_deg`、`unlock_yaw_error_deg`、`lock_residual_error_rate_deg_s`、`unlock_residual_error_rate_deg_s`、`search_enabled`、`search_yaw_rate_deg_s`、`command_interface`、`gimbal_yaw_joint_name`、`gimbal_pitch_joint_name`。
- `src/uav_trajectory_tracking/config/visual_interception.yaml`: 视觉拦截参数，例如 `truth_guidance_enabled`、`truth_guidance_required`、`pursuit_speed_mps`、`navigation_gain`、`max_guidance_accel_mps2`、`lock_loss_grace_s`、`coast_velocity_decay_s`、`yaw_mode` 和相机/云台光轴运动学常量。

## 风场

默认风场在 `src/uav_trajectory_tracking/config/wind.yaml`，坐标系是 Gazebo ENU：

- `linear_velocity_mps[0]`: 向东风速
- `linear_velocity_mps[1]`: 向北风速
- `linear_velocity_mps[2]`: 向上风速

默认值 `[3.0, 0.0, 0.0]` 是 3 m/s 向东水平风。风场使用 Gazebo 标准的
`<wind>` + `WindEffects` 结构，`wind_effects` 下的字段和 Gazebo SDF 标签保持一致；
其中 `time_for_rise` / `period` 单位是秒，方向扰动 `amplitude` 单位是弧度。
启动前改 YAML 会自动渲染到 Gazebo world；仿真运行中也可以临时改基础风速：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/set_wind.sh 3.0 0.0 0.0   # 3 m/s east wind
```

常用可复现实验风况放在 `src/uav_trajectory_tracking/config/wind_profiles/`：

- `calm.yaml`: 无风基线。
- `steady_east_3ms.yaml`: 3 m/s 向东恒定风。
- `gust_east_3ms.yaml`: 3 m/s 向东基础风，带轻微阵风，和默认配置一致。
- `gust_east_5ms.yaml`: 5 m/s 向东基础风，带更强阵风。

使用指定风况启动：

```bash
WIND_FILE=/home/zk/uav_trajectory_tracking_sim/src/uav_trajectory_tracking/config/wind_profiles/gust_east_5ms.yaml \
  ./scripts/start_px4_gazebo.sh
```

监听 Gazebo 运行时风速控制消息：

```bash
./scripts/watch_wind.sh
```

## 轨迹日志

每次启动 `start_trajectory_tracking.sh` 时默认会同步记录两份 CSV 轨迹日志，目录为
`log/trajectory_runs/<启动时间>/`：

- `px4_estimate.csv`: PX4 EKF/控制侧看到的估计状态，坐标系为 PX4 local NED，机体系为 FRD。
- `gazebo_truth.csv`: Gazebo 物理真值 odometry，原始坐标系为 Gazebo ENU，同时写入 NED 等效列便于和 PX4 对比。

真值日志由 `x500_gimbal_trajectory_wind` 内的 Gazebo `OdometryPublisher` 发布
`/model/x500_0/odometry_with_covariance`，再通过 `ros_gz_bridge` 桥接到 ROS 2。PX4 估计日志订阅
`/fmu/out/vehicle_local_position_v1`、`/fmu/out/vehicle_attitude` 和
`/fmu/out/vehicle_odometry`。Gazebo 真值 CSV 中的加速度由真值速度差分得到；
PX4 估计 CSV 中的加速度直接来自 `VehicleLocalPosition.ax/ay/az`。
两份 CSV 都保留 `ros_time_s` / `ros_elapsed_s`；其中 `ros_time_s` 是写入时的系统时间，
`ros_elapsed_s` 是 logger 启动后的相对时间。PX4 表额外写入 `px4_time_s` 和
`px4_elapsed_s`，Gazebo 表额外写入 `gazebo_time_s` 和 `gazebo_elapsed_s`。
画图和误差分析优先使用 `px4_elapsed_s` / `gazebo_elapsed_s` 作为横轴。

可以指定日志根目录或本次运行名：

```bash
LOG_ROOT=/home/zk/uav_logs RUN_ID=wind_3ms_figure8 ./scripts/start_trajectory_tracking.sh
```

不需要 CSV 记录时：

```bash
./scripts/start_trajectory_tracking.sh enable_csv_logging:=false
```

## 可视化话题

- `/trajectory_markers`: 轨迹起点、终点和当前飞行位置。
- `/trajectory_path`: YAML 参数化曲线采样得到的规划轨迹。
- `/vehicle_path`: 飞行过程中累积的实际轨迹。
- `/trajectory_tracker/current_stage`: 当前轨迹阶段，`0=entry`、`1=trajectory`、`2=return`、`3=finished`。
- `/x500_0/camera/image_raw`: 主机云台相机原始图像。
- `/x500_0/camera/camera_info`: 主机云台相机内参。
- `/x500_0/yolo/tracks`: YOLO + BoT-SORT 跟踪框，类型为 `vision_msgs/Detection2DArray`，其中 `Detection2D.id` 是跨帧 track id。
- `/x500_0/yolo/tracks_image`: YOLO + BoT-SORT 标注后的图像。
- `/x500_0/gimbal/joint_states`: Gazebo 云台关节反馈，`gimbal_target_tracker` 用它计算 `actual_yaw/actual_pitch`。
- `/x500_0/gimbal_target_tracker/error`: 云台视觉伺服视线角误差，`vector.x/y` 分别为 yaw/pitch 角误差，单位为 degree。
- `/x500_0/gimbal_target_tracker/residual_error_rate`: 云台残余图像误差角速度，`x/y` 为 yaw/pitch 残差角速度，单位为 degree/s，`z` 为 `0..1` 锁定质量。
- `/x500_0/gimbal_target_tracker/tracking_active`: 云台节点是否收到新鲜目标跟踪结果。
- `/x500_0/gimbal_target_tracker/lock_active`: 目标是否居中且稳定到足以作为外层导引门控。
- `/x500_0/gimbal_target_tracker/state`: 云台控制诊断，包含状态机状态、`cmd_yaw/cmd_pitch`、`actual_yaw/actual_pitch`、锁定阈值、残差角速度、积分项、反馈年龄和搜索状态。
- `/fmu/in/gimbal_manager_set_attitude`: 云台高频姿态 setpoint，类型为 `px4_msgs/msg/GimbalManagerSetAttitude`。
- `/model/x500_0/odometry_with_covariance`: 主机 Gazebo truth odometry，`visual_pursuit_interceptor` 通过 `ros_gz_bridge` 订阅后转成 NED。
- `/model/x500_1/odometry_with_covariance`: 目标机 Gazebo truth odometry，视觉拦截导引用它计算真实相对位置/速度。
- `/x500_0/visual_pursuit_interceptor/diagnostics`: 视觉拦截诊断，包含 `state`、`pursuing`、`velocity_control_active`、真实 `range_m`、`closing_speed_mps`、`relative_*`、`los_rate_*`、`gimbal_truth_los_error_deg` 和输出速度。
- `/target/trajectory_markers`: 目标无人机轨迹可视化。
- `/target/trajectory_path`: 目标无人机规划路径。
- `/target/vehicle_path`: 目标无人机实际轨迹。
- `/target/trajectory_tracker/current_stage`: 目标无人机当前轨迹阶段，编号含义同主机。

## 常见检查

如果 `x500_0` 不起飞，先确认：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_trajectory_tracking_sim/install/setup.bash
ros2 topic info /fmu/out/vehicle_status_v4 --verbose
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --qos-durability transient_local --once
```

如果没有跟踪结果，检查：

```bash
ros2 topic echo /x500_0/camera/image_raw --once
ros2 topic echo /x500_0/camera/camera_info --once
ros2 topic echo /x500_0/yolo/tracks --once
```

并确认：

- `yolov8s.pt` 是否存在于仓库根目录，或通过 `YOLO_WEIGHTS_PATH` 指定正确路径。
- `src/uav_trajectory_tracking/config/yolo_tracking.yaml` 中的 `classes` 是否过滤掉目标类别。
- `confidence_threshold` 是否过高。
- 相机图像桥接是否开启：`ENABLE_CAMERA_BRIDGE=true`。

如果云台不跟踪，检查：

```bash
ros2 topic echo /x500_0/gimbal_target_tracker/error --once
ros2 topic echo /x500_0/gimbal_target_tracker/tracking_active --once
ros2 topic echo /x500_0/gimbal_target_tracker/lock_active --once
ros2 topic echo /x500_0/gimbal_target_tracker/state --once
ros2 topic echo /x500_0/gimbal/joint_states --once
ros2 topic echo /fmu/in/gimbal_manager_set_attitude --once
```

并确认：

- `ENABLE_GIMBAL_TRACKING=true`。
- `GIMBAL_INPUT_TOPIC` 与 `YOLO_TRACKS_TOPIC` 一致。
- `/x500_0/gimbal/joint_states` 有 `cgo3_vertical_arm_joint` 和 `cgo3_camera_joint` 关节反馈；如果没有该 topic，需要重启 `scripts/start_px4_gazebo.sh`，让 Gazebo 加载本仓库云台模型中的 `JointStatePublisher` 插件。
- `/fmu/in/gimbal_manager_set_attitude` 有持续 setpoint；如果没有该 topic，确认 `/home/zk/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 已包含 `/fmu/in/gimbal_manager_set_attitude`，然后重新启动 `scripts/start_px4_gazebo.sh` 让 PX4 重新生成 XRCE-DDS topic。
- `src/uav_trajectory_tracking/config/gimbal_tracking.yaml` 中的 `target_class_id`、`target_track_id`、`min_score`、bbox 尺寸过滤阈值是否合理。
- 云台方向反了时，修改 `yaw_error_sign` 或 `pitch_error_sign`。
- 目标稳定偏离画面中心时，可小幅增加 `yaw_ki_s_inv2` 或 `pitch_ki_s_inv2`；出现慢速漂移时先确认 `*_feedforward_deg_s` 是否为 0。

如果视觉拦截不进入 `pursuit`，检查：

```bash
ros2 topic echo /x500_0/gimbal_target_tracker/lock_active --once
ros2 topic echo /model/x500_0/odometry_with_covariance --once
ros2 topic echo /model/x500_1/odometry_with_covariance --once
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

并确认：

- `start_visual_interception.sh` 已启动，或 `visual_interception.launch.py` 的 `enable_truth_odometry_bridge:=true`。
- `visual_interception.yaml` 中 `truth_guidance_required: true` 时，两个 truth odometry 话题都必须新鲜，否则节点不会进入 `pursuit`。
- `lock_active=false` 时先看 `/x500_0/gimbal_target_tracker/state` 里的 `last_image_yaw_error_deg`、`last_image_pitch_error_deg`、`lock_centered` 和 `lock_residual_rate_ok`。
- 短暂掉锁时状态应进入 `coast_on_lock_loss`，继续发布 velocity setpoint 衰减速度；只有长时间丢锁才切 position hold。

## World 文件说明

`/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/trajectory_tracking.sdf`
需要只包含基础世界、预加载的 `x500_0` / `x500_1` 和可选风场，
并加载 `AirSpeed`、`NavSat`、`Magnetometer`、`WindEffects` 等系统插件。该世界还必须包含
`spherical_coordinates`，否则 Gazebo 的 NavSat/GNSS 和磁场基准会异常，PX4 可能出现
安全检查失败或起飞后高度估计发散。修改世界文件后必须重启 PX4/Gazebo。

本仓库的 `px4_overlays/worlds/trajectory_tracking.sdf` 是基础世界，只放 Gazebo/PX4
必须的物理、传感器、地面、云台主机 `x500_0`、普通目标机 `x500_1` 和地理基准；
轨迹线、起降垫、边界和初始风向箭头只在传入 `SHOW_TRAJECTORY_VISUALS=true` 时由
`scripts/render_trajectory_world.py` 渲染到
`build/generated/worlds/trajectory_tracking.sdf`，再由 `scripts/start_px4_gazebo.sh`
同步到 PX4 的 Gazebo worlds 目录。
