# Runbook

## 快速检查

```bash
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_waypoint_tracking_sim/install/setup.bash
ros2 topic list | rg '/fmu/'
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --qos-durability transient_local --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --qos-durability transient_local --once
```

## 可视化检查

```bash
ros2 topic list | rg 'waypoint|vehicle_path'
ros2 topic echo /waypoint_tracker/current_waypoint_index --once
ros2 topic echo /waypoint_path --once
```

主机云台相机和 YOLO 检测启动后：

```bash
ros2 topic list | rg 'x500_0/(camera|yolo)'
ros2 topic echo /x500_0/yolo/detections --once
```

目标无人机启动后，按官方多机规则会出现在 `/px4_1/...` namespace：

```bash
ros2 topic list | rg '/px4_1/fmu/'
ros2 topic echo /px4_1/fmu/out/vehicle_status_v1 --qos-reliability best_effort --qos-durability transient_local --once
ros2 topic echo /target/waypoint_tracker/current_waypoint_index --once
```

RViz 使用 `map` 作为固定坐标系。`waypoint_visualizer` 会把 PX4 本地 NED 航点转换成 ROS ENU，
所以 RViz 中高度向上。Gazebo 世界坐标也是 ENU，3D 航点标记使用
`Gazebo x=east=NED y`、`Gazebo y=north=NED x`、`Gazebo z=up=-NED z`。

`x500_0` 是带云台相机的主机，Gazebo 模型名仍保持 `x500_0`，底层模型来自 PX4 官方
`x500_gimbal`。PX4 0 由 `scripts/start_px4_gazebo.sh` 以 `gz_x500_gimbal` airframe 启动；
目标机 `x500_1` 继续使用普通 `gz_x500` / `PX4_SYS_AUTOSTART=4001`。

Gazebo 静态标记不需要手工改 world。标准流程是只改
`src/uav_waypoint_tracking/config/waypoints.yaml`，然后重启 PX4/Gazebo：

```bash
./scripts/start_px4_gazebo.sh
./scripts/start_waypoint_tracking.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_waypoint_tracking.sh
```

`start_px4_gazebo.sh` 会调用 `scripts/render_waypoint_world.py`，把 YAML 航点转换成
`build/generated/worlds/waypoint_tracking.sdf`，再同步到
`/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/waypoint_tracking.sdf`。
如果使用自定义 YAML，PX4/Gazebo 终端和 ROS 终端都传入同一个变量：

```bash
WAYPOINTS_FILE=/home/zk/my_waypoints.yaml ./scripts/start_px4_gazebo.sh
WAYPOINTS_FILE=/home/zk/my_waypoints.yaml ./scripts/start_waypoint_tracking.sh
```

## 常见问题

1. 看不到 `/fmu/out/*`

确认 `MicroXRCEAgent udp4 -p 8888` 已启动，并且 PX4 SITL 日志里 `uxrce_dds_client` 正常连接。
当前 PX4 main 的部分输出话题带 `_v1` 后缀，tracker 默认订阅 `/fmu/out/vehicle_status_v1`
和 `/fmu/out/vehicle_local_position_v1`。

2. 节点切不进 Offboard

Offboard 模式要求先持续发送 setpoint。该节点默认先发送 1.5 秒 setpoint，再请求 Offboard 和解锁。

3. 无人机不按预期方向飞

航点使用 PX4 本地 NED，不是 ROS ENU。`z=-5` 表示离起飞点高度 5 m。
RViz 显示时已经转换为 ENU；不要把 RViz 里的 `z=5` 反写回 YAML。
Gazebo 的 3D 标记由脚本自动完成 NED 到 Gazebo 坐标的转换，不需要手工同步 SDF。
如果 Gazebo 里的航点标记相对实际飞行轨迹旋转 90 度，优先检查生成脚本是否使用了
`x=east=NED y, y=north=NED x` 的转换。

4. `x500_0` 不起飞

确认航点仿真启动脚本使用的是 `waypoint_tracking` 世界，而不是 `/home/zk/gimbal_track`
依赖的 `default1` 世界。`waypoint_tracking.sdf` 需要加载 `gz-sim-navsat-system`
和 `gz-sim-magnetometer-system`，并且不包含 `x500_target_moving`。`x500_0` 应包含
`x500_gimbal_waypoint_wind`，`x500_1` 应包含 `x500_waypoint_wind`。同时必须包含
`spherical_coordinates`，这是 NavSat/GNSS 和 Gazebo 磁场模型生成有效量测的基准。
如果 `/fmu/out/vehicle_local_position_v1` 里 `xy_valid: false`，PX4 没有可用水平位置估计，
Offboard 航点仿真不会起飞。若 `estimator_status_flags` 里 `cs_mag_field_disturbed: true`，
先检查 Gazebo 磁罗盘话题是否是约 `0.2-0.5` Gauss 的量级，不要再在 PX4 GZBridge 中额外
乘 `10000`。修改世界或模型文件后需要完整重启 PX4/Gazebo。

5. 起飞后一直向上飞

先确认航点没有写错：PX4 本地 NED 里 `z=-5` 表示上升到 5 m，而不是下降。若航点正确但
Gazebo 中高度持续增大，通常是状态估计输入和 Gazebo 真值脱节。重点检查：

```bash
gz topic -e -n 1 -t /world/waypoint_tracking/model/x500_0/link/base_link/sensor/navsat_sensor/navsat
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /fmu/out/estimator_status_flags --once
```

正常状态下 NavSat 应输出有效经纬高，`xy_valid/z_valid` 应为 `true`，
`cs_yaw_align/cs_gnss_pos` 应为 `true`，`cs_mag_field_disturbed` 应为 `false`。

6. RViz 看不到航点

确认 `start_waypoint_tracking.sh` 已经启动，或者单独运行：

```bash
ros2 run uav_waypoint_tracking waypoint_visualizer
```

RViz 的 Fixed Frame 需要是 `map`，并添加 `MarkerArray` 订阅 `/waypoint_markers`、
`Path` 订阅 `/waypoint_path` 和 `/vehicle_path`。仓库提供的 `scripts/start_rviz.sh`
已经预配置这些显示项。
