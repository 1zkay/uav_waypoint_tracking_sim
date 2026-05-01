# UAV Waypoint Tracking Simulation

PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy 的本地 NED 航点跟踪仿真工作区。

## 环境

- PX4: `/home/zk/PX4-Autopilot`
- Python venv: `/home/zk/px4-venv`
- ROS 2: Jazzy
- Gazebo: Harmonic
- DDS Agent: `MicroXRCEAgent udp4 -p 8888`

## 构建

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/build.sh
```

## 启动

终端 1:

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_agent.sh
```

终端 2:

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_px4_gazebo.sh
```

终端 3:

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_waypoint_tracking.sh
```

终端 4 可选，启动第二架目标无人机的 PX4 实例：

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_target_px4_gazebo.sh
```

终端 5 可选，让目标无人机也按同一份航点飞行：

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_target_waypoint_tracking.sh
```

终端 6 可选，用于查看航点、目标点、规划路径和实际飞行尾迹：

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/start_rviz.sh
```

当前 PX4 main 在这台机器上发布的是 `/fmu/out/vehicle_status_v1` 和
`/fmu/out/vehicle_local_position_v1`，启动文件默认已经使用这两个话题。
`start_waypoint_tracking.sh` 会同时启动控制节点和 RViz 可视化发布节点。

第二架目标无人机按 PX4 Gazebo 官方多机方式运行：每架机一个独立 PX4 SITL 实例。
主机使用默认实例 `px4_instance=0`、Gazebo 模型 `x500_0`、PX4 官方
`gz_x500_gimbal` airframe、ROS 2 话题 `/fmu/...`、`MAV_SYS_ID=1`。目标机使用
`scripts/start_target_px4_gazebo.sh` 启动为
`px4_instance=1`，连接 Gazebo 中预加载的 `x500_1`；PX4 会自动设置
`MAV_SYS_ID=2`、`UXRCE_DDS_KEY=2`，ROS 2 话题带 `/px4_1/...` namespace。
因此目标机的 Offboard 控制必须向 `/px4_1/fmu/in/...` 发布，并把
`VehicleCommand.target_system` 设为 `2`。`scripts/start_target_waypoint_tracking.sh`
已经按这个规则配置。`px4_instance > 0` 时不要复用主机的 `/fmu/in/*` 话题。

默认航点在 `src/uav_waypoint_tracking/config/waypoints.yaml`，坐标系是 PX4 本地 NED：

- `x`: 北/前
- `y`: 东/右
- `z`: 下，起飞点上方 5 m 写作 `-5.0`

只需要修改这个 YAML。`scripts/start_px4_gazebo.sh` 会在启动前自动根据同一个 YAML
生成 Gazebo 航点标记 world，并同步到 PX4 的 worlds 目录；`scripts/start_waypoint_tracking.sh`
也会默认把同一个 YAML 传给控制节点和 RViz 可视化节点。不要手工维护两份航点。

使用自定义航点文件时，PX4/Gazebo 终端和 ROS 终端都传入同一个变量：

```bash
WAYPOINTS_FILE=/home/zk/my_waypoints.yaml ./scripts/start_px4_gazebo.sh
WAYPOINTS_FILE=/home/zk/my_waypoints.yaml ./scripts/start_waypoint_tracking.sh
```

Gazebo 如果暂停，点击左下角播放按钮。航点仿真默认使用
`waypoint_tracking` 世界，预加载主机 `x500_0` 和目标机 `x500_1`。`x500_0` 使用本仓库的
`x500_gimbal_waypoint_wind` 包装模型，内部复用 PX4 官方 `x500_gimbal`；`x500_1` 仍使用普通
`x500_waypoint_wind`。两个机体都由 world 预加载，PX4 启动后分别通过
`PX4_GZ_MODEL_NAME=x500_0` 和 `PX4_GZ_MODEL_NAME=x500_1` 连接，因此可以只在本仓库内启用
受风模型和真值 odometry，不修改 PX4 原始 `x500_base` / `x500_gimbal`。
`default1` 保留给 `/home/zk/gimbal_track` 使用，其中仍包含 `x500_target_moving`。
Gazebo 世界中包含航点柱、设定高度航线、起降垫、边界和少量参照物；RViz 使用 ROS ENU
`map` 坐标显示相同航点，其中 PX4 NED 会自动转换为 `x=east, y=north, z=up`。
Gazebo 世界坐标同样按 ENU 显示：`Gazebo x=east`、`Gazebo y=north`、`Gazebo z=up`。

默认风场在 `src/uav_waypoint_tracking/config/wind.yaml`，坐标系是 Gazebo ENU：

- `linear_velocity_mps[0]`: 向东风速
- `linear_velocity_mps[1]`: 向北风速
- `linear_velocity_mps[2]`: 向上风速

默认值 `[3.0, 0.0, 0.0]` 是 3 m/s 向东水平风。风场使用 Gazebo 标准的
`<wind>` + `WindEffects` 结构，`wind_effects` 下的字段和 Gazebo SDF 标签保持一致；
其中 `time_for_rise` / `period` 单位是秒，方向扰动 `amplitude` 单位是弧度。
启动前改 YAML 会自动渲染到 Gazebo world；仿真运行中也可以临时改基础风速：

```bash
cd /home/zk/uav_waypoint_tracking_sim
./scripts/set_wind.sh 3.0 0.0 0.0   # 3 m/s east wind
```

常用可复现实验风况放在 `src/uav_waypoint_tracking/config/wind_profiles/`：

- `calm.yaml`: 无风基线。
- `steady_east_3ms.yaml`: 3 m/s 向东恒定风。
- `gust_east_3ms.yaml`: 3 m/s 向东基础风，带轻微阵风，和默认配置一致。
- `gust_east_5ms.yaml`: 5 m/s 向东基础风，带更强阵风。

使用指定风况启动：

```bash
WIND_FILE=/home/zk/uav_waypoint_tracking_sim/src/uav_waypoint_tracking/config/wind_profiles/gust_east_5ms.yaml \
  ./scripts/start_px4_gazebo.sh
```

监听 Gazebo 运行时风速控制消息：

```bash
./scripts/watch_wind.sh
```

每次启动 `start_waypoint_tracking.sh` 时默认会同步记录两份 CSV 轨迹日志，目录为
`log/trajectory_runs/<启动时间>/`：

- `px4_estimate.csv`: PX4 EKF/控制侧看到的估计状态，坐标系为 PX4 local NED，机体系为 FRD。
- `gazebo_truth.csv`: Gazebo 物理真值 odometry，原始坐标系为 Gazebo ENU，同时写入 NED 等效列便于和 PX4 对比。

真值日志由 `x500_gimbal_waypoint_wind` 内的 Gazebo `OdometryPublisher` 发布
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
LOG_ROOT=/home/zk/uav_logs RUN_ID=wind_3ms_figure8 ./scripts/start_waypoint_tracking.sh
```

不需要 CSV 记录时：

```bash
./scripts/start_waypoint_tracking.sh enable_csv_logging:=false
```

可视化话题：

- `/waypoint_markers`: 航点球、编号、接受半径、当前目标箭头。
- `/waypoint_path`: YAML 航点连成的规划路径。
- `/vehicle_path`: 飞行过程中累积的实际轨迹。
- `/waypoint_tracker/current_waypoint_index`: 当前目标航点索引。
- `/target/waypoint_markers`: 目标无人机航点可视化。
- `/target/waypoint_path`: 目标无人机规划路径。
- `/target/vehicle_path`: 目标无人机实际轨迹。
- `/target/waypoint_tracker/current_waypoint_index`: 目标无人机当前航点索引。

如果 `x500_0` 不起飞，先确认：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_waypoint_tracking_sim/install/setup.bash
ros2 topic info /fmu/out/vehicle_status_v1 --verbose
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --qos-durability transient_local --once
```

`/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/waypoint_tracking.sdf`
需要只包含基础世界、预加载的 `x500_0` / `x500_1`、由 YAML 自动生成的静态航点标记和可选风场，
并加载 `AirSpeed`、`NavSat`、`Magnetometer`、`WindEffects` 等系统插件。该世界还必须包含
`spherical_coordinates`，否则 Gazebo 的 NavSat/GNSS 和磁场基准会异常，PX4 可能出现
安全检查失败或起飞后高度估计发散。修改世界文件后必须重启 PX4/Gazebo。

本仓库的 `px4_overlays/worlds/waypoint_tracking.sdf` 是基础世界，只放 Gazebo/PX4
必须的物理、传感器、地面、云台主机 `x500_0`、普通目标机 `x500_1` 和地理基准；
航点柱、接受半径、规划航线、起降垫、边界和初始风向箭头由
`scripts/render_waypoint_world.py` 自动渲染到
`build/generated/worlds/waypoint_tracking.sdf`，再由 `scripts/start_px4_gazebo.sh`
同步到 PX4 的 Gazebo worlds 目录。
