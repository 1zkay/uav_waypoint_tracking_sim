# Visual Pursuit Interception

本文档说明 `visual_pursuit_interceptor` 中相机光轴到 PX4 速度 setpoint 的运动学约定。

## 标准坐标约定

- PX4 本地位置和 `TrajectorySetpoint` 使用 NED：`X north, Y east, Z down`。
- PX4 机体系使用 FRD：`X forward, Y right, Z down`。
- Gazebo 模型/link 约定遵循 REP 103 风格的 FLU：`X forward, Y left, Z up`。
- ROS camera optical frame 的常见约定是 `Z forward, X right, Y down`，但 Gazebo rendering camera sensor 的物理传感器 frame 仍按 Gazebo 机器人 frame 处理，本模型中取 `+X` 为相机 boresight。

因此，拦截器先在 Gazebo/SDF 的 FLU 链路中算出相机 sensor `+X` 光轴，再转换为 PX4 body FRD：

```text
v_frd = [v_flu.x, -v_flu.y, -v_flu.z]
```

然后使用 PX4 `VehicleAttitude.q` 把 body FRD 向量旋转到 local NED，作为追踪速度方向。

## 当前 SDF 链路

`x500_gimbal_self_filtered` 把 `gimbal_self_hidden` 固定到 `base_link`，并给云台 include 施加 `yaw=pi`：

```text
base_link
  -> gimbal include pose yaw=pi
  -> cgo3_mount_link
  -> yaw joint:   cgo3_vertical_arm_joint, axis [0, 0, -1]
  -> roll joint:  cgo3_horizontal_arm_joint, axis [-1, 0, 0]
  -> pitch joint: cgo3_camera_joint, axis [0, 1, 0]
  -> camera_link
  -> camera sensor pose yaw=pi
  -> camera sensor +X optical axis
```

这两处 `yaw=pi` 在零关节角时相互抵消，所以零位光轴仍指向机体前方。以前的简化公式在 roll=0 时与这个特定 SDF 大致等价，但它没有显式表达 mount pose、roll 关节、sensor pose 和 FLU->FRD 变换，容易在模型变更后静默失效。

## 实现方式

`visual_pursuit_interceptor.py` 现在使用显式旋转链：

```text
R_base_sensor =
  R_mount_rpy
  * R_axis(yaw_axis, yaw)
  * R_axis(roll_axis, roll)
  * R_axis(pitch_axis, pitch)
  * R_sensor_rpy

los_flu = R_base_sensor * camera_optical_axis_sensor
los_body_frd = FLU_to_FRD(los_flu)
los_ned = R_body_to_ned(vehicle_attitude.q) * los_body_frd
```

这些 SDF 常量都写在 `src/uav_trajectory_tracking/config/visual_interception.yaml`：

```yaml
gimbal_mount_rpy_rad: [0.0, 0.0, 3.141592653589793]
gimbal_yaw_axis: [0.0, 0.0, -1.0]
gimbal_roll_axis: [-1.0, 0.0, 0.0]
gimbal_pitch_axis: [0.0, 1.0, 0.0]
camera_sensor_rpy_rad: [0.0, 0.0, 3.141592653589793]
camera_optical_axis_sensor: [1.0, 0.0, 0.0]
```

`min_body_forward_component` 默认是 `0.0`，表示严格按真实光轴追踪。若为了飞行包线测试需要人为保持前向分量，也只在 body FRD 下钳制 `X forward`，不再在 NED 下钳制 `X north`。
