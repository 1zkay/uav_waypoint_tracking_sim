# Visual Pursuit Interception

本文档说明当前 `visual_pursuit_interceptor` 的实际实现。当前链路用于仿真验证：云台视觉伺服提供导引头锁定门控，Gazebo truth odometry 提供真实相对位置和相对速度，拦截器使用 LOS-rate 比例导引发布 PX4 NED velocity setpoint。

## 控制分层

当前实现按以下层次拆分：

```text
YOLO + BoT-SORT
  -> gimbal_target_tracker: 图像误差视觉伺服，把目标拉到相机中心
  -> seeker lock: tracking_active / lock_active
  -> visual_pursuit_interceptor: truth LOS / LOS-rate PN 导引
  -> PX4 Offboard velocity 或 position setpoint
  -> PX4 速度、姿态和电机内环
```

`tracking_active` 只表示有新鲜目标检测/跟踪；`lock_active` 才表示云台导引头已经把目标居中并且残差稳定。拦截器只在 `lock_active` 新鲜、云台反馈新鲜、truth odometry 新鲜时进入 `pursuit`。

## 启动入口

常规顺序：

```bash
./scripts/start_agent.sh
./scripts/start_px4_gazebo.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_trajectory_tracking.sh
./scripts/start_visual_interception.sh
```

`start_visual_interception.sh` 会启动相机桥接、YOLO + BoT-SORT、云台视觉伺服、truth odometry bridge 和视觉拦截节点。底层 launch 为：

```bash
ros2 launch uav_trajectory_tracking visual_interception.launch.py
```

视觉拦截时不要同时运行主机 `start_trajectory_tracking.sh`，否则 `trajectory_tracker` 和 `visual_pursuit_interceptor` 会同时向 `/fmu/in/trajectory_setpoint` 发布 setpoint。

默认 truth odometry 话题：

```text
/model/x500_0/odometry_with_covariance
/model/x500_1/odometry_with_covariance
```

`visual_interception.launch.py` 默认 `enable_truth_odometry_bridge:=true`，会通过 `ros_gz_bridge` 把 Gazebo `gz.msgs.OdometryWithCovariance` 桥接成 ROS 2 `nav_msgs/msg/Odometry`。

## 状态机

拦截器状态如下：

```text
initializing
takeoff
transit_to_hover
hold
gimbal_search
acquiring_target
pursuit
coast_on_lock_loss
target_lost
```

主要转移逻辑：

```text
vehicle/gimbal/truth 未准备好
  -> initializing / hold / target_lost

初始高度和悬停点到达
  -> hold

tracking_active 或 lock_active 新鲜，但未满足追击条件
  -> acquiring_target

lock_active 新鲜，truth odometry 新鲜，gimbal feedback 新鲜
  -> pursuit

pursuit 后短暂掉锁，且仍在 lock_loss_grace_s 内
  -> coast_on_lock_loss

长时间掉锁或检测丢失
  -> target_lost，捕获当前位置并等待云台搜索

云台状态为 local_search/global_search 且垂直搜索开启
  -> gimbal_search，固定 XY 并按配置上下扫描高度
```

`pursuit` 和 `coast_on_lock_loss` 都使用 PX4 velocity control：

```text
OffboardControlMode.position = false
OffboardControlMode.velocity = true
TrajectorySetpoint.position = [NaN, NaN, NaN]
TrajectorySetpoint.velocity = [vx, vy, vz]
```

`hold`、`gimbal_search`、`acquiring_target` 和 `target_lost` 使用 PX4 position control。短暂掉锁不会立刻切 position hold，避免 PX4 往旧 hold 点回拉。默认配置下，云台通过 `/x500_0/gimbal_target_tracker/search_active` 报告正在执行 `local_search` 或 `global_search` 时，主机无人机会围绕丢失位置做 NED z 方向上下扫描，以弥补云台 pitch 视场不足。

## Truth 相对几何

Gazebo odometry 是 ENU，拦截器转成 PX4 NED：

```text
x_ned = y_enu
y_ned = x_enu
z_ned = -z_enu

vx_ned = vy_enu
vy_ned = vx_enu
vz_ned = -vz_enu
```

真实相对状态：

```text
relative_position_ned = target_position_ned - host_position_ned
relative_velocity_ned = target_velocity_ned - host_velocity_ned
range_m = norm(relative_position_ned)
los_ned = normalize(relative_position_ned)
closing_speed_mps = -dot(relative_velocity_ned, los_ned)
```

LOS rate 使用刚体相对运动公式：

```text
los_rate_ned = cross(los_ned, relative_velocity_ned) / range_m
```

这个 truth 几何只用于仿真导引验证。云台光轴仍然计算并发布诊断，用于检查视觉导引头是否真正对准目标。

## 比例导引

当前导引是 LOS-rate PN 风格的 velocity setpoint 生成器：

```text
guidance_accel = N * Vc * cross(los_rate, los)
```

其中：

- `N` 对应 `navigation_gain`
- `Vc` 使用真实 `closing_speed_mps`，并限制为非负
- `guidance_accel` 由 `max_guidance_accel_mps2` 限幅

导引加速度积分成横向速度：

```text
lateral_velocity += guidance_accel * dt
lateral_velocity = reject_from_axis(lateral_velocity, los)
lateral_velocity = limit_norm(lateral_velocity, max_lateral_guidance_speed_mps)
```

最终速度指令：

```text
closing_velocity = pursuit_speed_mps * los
target_velocity_feedforward = target_truth_velocity_ned
velocity_cmd = target_velocity_feedforward + closing_velocity + lateral_velocity
```

`pursuit_speed_mps` 现在表示自身期望沿真实 LOS 增加的闭合速度，不再作为闭合速度估计。真实 `closing_speed_mps` 只进入 PN 加速度项。

输出速度经过：

- `max_vertical_speed_mps` 垂直限幅
- `max_pursuit_accel_mps2` 速度变化率限幅

## 掉锁滑行

`coast_on_lock_loss` 用来处理短暂视觉掉锁。它保持 velocity mode，并对上一帧速度做指数衰减：

```text
velocity_cmd = last_velocity_cmd * exp(-dt / coast_velocity_decay_s)
```

这样云台短时重捕获目标时，PX4 不会在 velocity 和 position 控制间来回切换。超过 `lock_loss_grace_s` 后才捕获当前位置并进入 position hold。

## 云台光轴诊断

虽然导引用 truth LOS，拦截器仍按 SDF 显式计算云台光轴：

```text
R_base_sensor =
  R_mount_rpy
  * R_axis(yaw_axis, yaw)
  * R_axis(roll_axis, roll)
  * R_axis(pitch_axis, pitch)
  * R_sensor_rpy

los_flu = R_base_sensor * camera_optical_axis_sensor
los_body_frd = [los_flu.x, -los_flu.y, -los_flu.z]
gimbal_los_ned = R_body_to_ned(vehicle_attitude.q) * los_body_frd
```

这些 SDF 常量来自 `src/uav_trajectory_tracking/config/visual_interception.yaml`：

```yaml
gimbal_mount_rpy_rad: [0.0, 0.0, 3.141592653589793]
gimbal_yaw_axis: [0.0, 0.0, -1.0]
gimbal_roll_axis: [-1.0, 0.0, 0.0]
gimbal_pitch_axis: [0.0, 1.0, 0.0]
camera_sensor_rpy_rad: [0.0, 0.0, 3.141592653589793]
camera_optical_axis_sensor: [1.0, 0.0, 0.0]
```

诊断中的 `gimbal_truth_los_error_deg` 是云台光轴和真实目标 LOS 的夹角。它不直接控制无人机，但可用来判断 `lock_active=true` 时导引头是否真的对准目标。

## 关键配置

`src/uav_trajectory_tracking/config/visual_interception.yaml`：

```yaml
pursuit_speed_mps: 1.2
navigation_gain: 3.0
max_guidance_accel_mps2: 1.5
max_lateral_guidance_speed_mps: 1.2
los_rate_filter_alpha: 0.35

truth_guidance_enabled: true
truth_guidance_required: true
truth_odometry_timeout_s: 0.5

lock_loss_grace_s: 0.6
coast_velocity_decay_s: 0.6
search_vertical_motion_enabled: true
search_vertical_amplitude_m: 2.0
search_vertical_period_s: 12.0
search_vertical_min_z_ned: -8.0
search_vertical_max_z_ned: -2.0
yaw_mode: fixed_north
```

`truth_guidance_required: true` 表示 truth odometry 不新鲜时禁止进入 `pursuit`。这是仿真验证模式下的保守设置，避免回退到不可信的云台光轴导引。

## 诊断话题

主要诊断：

```bash
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

常看字段：

- `state`
- `pursuing`
- `velocity_control_active`
- `detection_active`
- `lock_active`
- `lock_loss_age_s`
- `truth_feedback_fresh`
- `range_m`
- `closing_speed_mps`
- `relative_position_ned_*`
- `relative_velocity_ned_*`
- `los_ned_*`
- `truth_los_ned_*`
- `gimbal_los_ned_*`
- `gimbal_truth_los_error_deg`
- `los_rate_ned_*`
- `guidance_accel_ned_*`
- `velocity_ned_*`

典型正常追击：

```text
state: pursuit
pursuing: true
velocity_control_active: true
lock_active: true
truth_feedback_fresh: true
range_m: finite
closing_speed_mps: finite
```

短暂掉锁但不回拉：

```text
state: coast_on_lock_loss
pursuing: false
velocity_control_active: true
velocity_ned_*: 衰减
```

长时间丢失：

```text
state: target_lost
pursuing: false
velocity_control_active: false
hold_x_m/hold_y_m/hold_z_m: 丢失时当前位置
```

## 常见问题

如果一直不进入 `pursuit`：

1. 检查 `/x500_0/gimbal_target_tracker/lock_active` 是否为 `true`。
2. 检查 `/model/x500_0/odometry_with_covariance` 和 `/model/x500_1/odometry_with_covariance` 是否有输出。
3. 检查诊断中的 `truth_feedback_fresh`。
4. 检查 `/x500_0/gimbal_target_tracker/state` 中的 `lock_centered`、`lock_residual_rate_ok` 和图像误差。

如果接近目标后反复后退：

1. 诊断应确认短暂掉锁进入 `coast_on_lock_loss`，而不是直接 `target_lost`。
2. `lock_loss_grace_s` 太短会让 position hold 更早介入。
3. `lock_active` 抖动时先看云台端 unlock 阈值和 residual error rate。

如果 `gimbal_truth_los_error_deg` 很大但 `lock_active=true`：

1. 检查 Gazebo truth ENU 到 NED 的坐标转换是否与目标模型一致。
2. 检查云台 joint state 话题是否来自当前 `x500_0`。
3. 检查 `gimbal_mount_rpy_rad`、各关节轴和 `camera_sensor_rpy_rad` 是否与 SDF 一致。
