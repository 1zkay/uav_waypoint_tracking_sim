# 无人机与云台硬件参数汇总

本文档汇总当前工作区实际使用的仿真硬件参数，包括主机无人机 `x500_0`、目标无人机 `x500_1`、主机云台、云台相机，以及对应参数的修改路径。

这里的“硬件参数”指 Gazebo/PX4 SDF 模型、PX4 airframe 中定义的质量、惯量、旋翼、电机、传感器、云台机械结构和云台控制器参数；不包括 ROS 视觉伺服调参、YOLO 推理参数或航点控制算法参数。

## 当前模型链路

当前 Gazebo world 中预加载两架无人机：

| 角色       | Gazebo 模型名 | 模型 URI                              | 初始位姿        |
| ---------- | ------------- | ------------------------------------- | --------------- |
| 主机无人机 | `x500_0`    | `model://x500_gimbal_waypoint_wind` | `0 0 0 0 0 0` |
| 目标无人机 | `x500_1`    | `model://x500_waypoint_wind`        | `0 5 0 0 0 0` |

主机模型解析链路：

```text
x500_gimbal_waypoint_wind
  -> x500_gimbal
       -> x500
            -> x500_base
       -> gimbal
```

目标机模型解析链路：

```text
x500_waypoint_wind
  -> x500
       -> x500_base
```

本仓库中的两个 wrapper 模型都启用了风场和 Gazebo 三维真值里程计：

| 参数（中文说明） | 当前值                                  |
| --------------- | --------------------------------------- |
| `enable_wind`（是否启用风场受力） | `true`                                |
| 里程计插件      | `gz::sim::systems::OdometryPublisher` |
| 里程计维度      | `3`                                   |

## 无人机机体参数

以下参数来自 PX4 官方 Gazebo 模型 `x500_base` 和 `x500`。

### 机架与机身

| 参数（中文说明）         | 当前值                                      |
| ----------------------- | ------------------------------------------- |
| 基础模型位姿            | `0 0 .24 0 0 0`                           |
| `self_collide`（模型自身碰撞） | `false`                                   |
| `static`（是否静态模型） | `false`                                   |
| 机身 `base_link` 质量 | `2.0 kg`                                  |
| 机身惯量 `ixx`        | `0.02166666666666667 kg*m^2`              |
| 机身惯量 `iyy`        | `0.02166666666666667 kg*m^2`              |
| 机身惯量 `izz`        | `0.04000000000000001 kg*m^2`              |
| 机身视觉网格            | `model://x500_base/meshes/NXP-HGD-CF.dae` |
| 电机座视觉网格          | `model://x500_base/meshes/5010Base.dae`   |
| 电机钟视觉网格          | `model://x500_base/meshes/5010Bell.dae`   |

机身碰撞体：

| 碰撞体（中文说明）          | 位姿                         | 尺寸                                             |
| ------------------------- | ---------------------------- | ------------------------------------------------ |
| `base_link_collision_0`（机身中心主碰撞体） | `0 0 .007 0 0 0`           | `0.35355339059327373 0.35355339059327373 0.05` |
| `base_link_collision_1`（一侧起落架支撑） | `0 -0.098 -.123 -0.35 0 0` | `0.015 0.015 0.21`                             |
| `base_link_collision_2`（另一侧起落架支撑） | `0 0.098 -.123 0.35 0 0`   | `0.015 0.015 0.21`                             |
| `base_link_collision_3`（一侧起落架横杆） | `0 -0.132 -.2195 0 0 0`    | `0.25 0.015 0.015`                             |
| `base_link_collision_4`（另一侧起落架横杆） | `0 0.132 -.2195 0 0 0`     | `0.25 0.015 0.015`                             |

### 旋翼

每个旋翼 link 的通用参数：

| 参数（中文说明）  | 当前值                                                            |
| ---------------- | ----------------------------------------------------------------- |
| 旋翼质量         | `0.016076923076923075 kg`                                       |
| 旋翼惯量 `ixx` | `3.8464910483993325e-07 kg*m^2`                                 |
| 旋翼惯量 `iyy` | `2.6115851691700804e-05 kg*m^2`                                 |
| 旋翼惯量 `izz` | `2.649858234714004e-05 kg*m^2`                                  |
| 旋翼关节类型     | `revolute`                                                      |
| 旋翼关节轴       | `0 0 1`                                                         |
| 旋翼关节上下限   | `-1e+16 / 1e+16 rad`                                            |
| 螺旋桨视觉缩放   | `0.8461538461538461 0.8461538461538461 0.8461538461538461`      |
| 旋翼碰撞体尺寸   | `0.2792307692307692 0.016923076923076923 0.0008461538461538462` |

旋翼位置与方向：

| 旋翼（中文说明） | Link 位姿                    | 螺旋桨网格            | 电机方向 |
| ----------- | ---------------------------- | --------------------- | -------- |
| `rotor_0`（右后旋翼，Gazebo 坐标） | `0.174 -0.174 0.06 0 0 0`  | `1345_prop_ccw.stl` | `ccw`  |
| `rotor_1`（左前旋翼，Gazebo 坐标） | `-0.174 0.174 0.06 0 0 0`  | `1345_prop_ccw.stl` | `ccw`  |
| `rotor_2`（右前旋翼，Gazebo 坐标） | `0.174 0.174 0.06 0 0 0`   | `1345_prop_cw.stl`  | `cw`   |
| `rotor_3`（左后旋翼，Gazebo 坐标） | `-0.174 -0.174 0.06 0 0 0` | `1345_prop_cw.stl`  | `cw`   |

### 电机模型

四个电机都使用 Gazebo `gz::sim::systems::MulticopterMotorModel`，参数一致：

| 参数（中文说明）              | 当前值                                   |
| ---------------------------- | ---------------------------------------- |
| `timeConstantUp`（电机加速时间常数） | `0.0125 s`                             |
| `timeConstantDown`（电机减速时间常数） | `0.025 s`                              |
| `maxRotVelocity`（最大转速） | `1000.0 rad/s`                         |
| `motorConstant`（推力系数） | `8.54858e-06`                          |
| `momentConstant`（反扭矩系数） | `0.016`                                |
| `commandSubTopic`（电机速度指令 topic） | `command/motor_speed`                  |
| `rotorDragCoefficient`（旋翼阻力系数） | `8.06428e-05`                          |
| `rollingMomentCoefficient`（滚转力矩系数） | `1e-06`                                |
| `rotorVelocitySlowdownSim`（仿真转速缩放系数） | `10`                                   |
| `motorType`（电机控制类型） | `velocity`                             |
| 故障插件                     | `gz::sim::systems::MotorFailureSystem` |

### 质量汇总

| 模型                        | SDF 惯性项统计范围              | 近似总质量          |
| --------------------------- | ------------------------------- | ------------------- |
| `x500_1` 目标无人机       | 机身 + 4 个旋翼                 | `2.0643076923 kg` |
| `x500_0` 主机无人机带云台 | 机身 + 4 个旋翼 + 4 个云台 link | `2.4643076923 kg` |

上述质量是从 SDF inertial 项直接相加得到的近似值，不包含风场、控制器状态、PX4 估计器参数或运行时外力影响。

## 无人机传感器参数

基础 `x500` 传感器挂在 `base_link` 上：

| 传感器（中文说明）       | 类型             |     更新率 | 噪声/关键参数                                                                                  |
| ----------------------- | ---------------- | ---------: | ---------------------------------------------------------------------------------------------- |
| `air_pressure_sensor`（气压计） | `air_pressure` |  `50 Hz` | 气压高斯噪声 `stddev=3`                                                                      |
| `magnetometer_sensor`（磁力计） | `magnetometer` | `100 Hz` | x/y/z 高斯噪声 `stddev=0.0001`                                                               |
| `imu_sensor`（机体 IMU） | `imu`          | `250 Hz` | 陀螺 x/y/z `stddev=0.0008726646`；加速度 x/y `stddev=0.00637`；加速度 z `stddev=0.00686` |
| `navsat_sensor`（GNSS/NavSat） | `navsat`       |  `30 Hz` | 模型中未显式设置传感器噪声                                                                     |

world 中同时加载了这些传感器需要的 Gazebo 系统插件：`Imu`、`AirPressure`、`AirSpeed`、`NavSat`、`Magnetometer` 和 `Sensors`，渲染引擎为 `ogre2`。

## 云台安装参数

主机 `x500_0` 使用 PX4 官方 `x500_gimbal` 模型。云台安装参数：

| 参数（中文说明）                   | 当前值                |
| --------------------------------- | --------------------- |
| 云台模型 URI                      | `model://gimbal`    |
| 云台相对 `x500` 的 include 位姿 | `0 0 0.26 0 0 3.14` |
| 固定连接关节                      | `GimbalAttachJoint` |
| 关节类型                          | `fixed`             |
| 父 link                           | `base_link`         |
| 子 link                           | `cgo3_mount_link`   |

## 云台物理参数

云台模型为非静态模型，`self_collide=false`。

以下四个云台 link 的质量均为 `0.1 kg`，惯量均为 `ixx=iyy=izz=0.001 kg*m^2`：

| Link（中文说明）              | 惯性位姿                      |
| ---------------------------- | ----------------------------- |
| `cgo3_mount_link`（云台安装座） | `-0.02552 0 -0.08136 0 0 0` |
| `cgo3_vertical_arm_link`（云台垂直臂） | `0 0 -0.1283 0 0 0`         |
| `cgo3_horizontal_arm_link`（云台水平臂） | `-0.0213 0 -0.162 0 0 0`    |
| `camera_link`（相机主体） | `-0.0412 0 -0.162 0 0 0`    |

相机碰撞体：

| 参数（中文说明）     | 当前值      |
| ------------------- | ----------- |
| 碰撞体形状          | 球体        |
| 半径                | `0.035 m` |
| 摩擦系数 `mu/mu2` | `1 / 1`   |
| 接触刚度 `kp`     | `1e+8`    |
| 接触阻尼 `kd`     | `1`       |
| 接触 `max_vel`（最大接触修正速度） | `0.01`    |
| 接触 `min_depth`（最小接触深度） | `0.001`   |

## 云台关节参数

| 轴    | 关节                          | 父 link -> 子 link                                         | 轴向       | 限位                              |  Effort | Velocity | Damping |
| ----- | ----------------------------- | ---------------------------------------------------------- | ---------- | --------------------------------- | ------: | -------: | ------: |
| yaw   | `cgo3_vertical_arm_joint`   | `cgo3_mount_link` -> `cgo3_vertical_arm_link`          | `0 0 -1` | `-1e+16` 到 `1e+16 rad`       | `100` |   `-1` | `0.1` |
| roll  | `cgo3_horizontal_arm_joint` | `cgo3_vertical_arm_link` -> `cgo3_horizontal_arm_link` | `-1 0 0` | `-0.785398` 到 `0.785398 rad` | `100` |   `-1` | `0.1` |
| pitch | `cgo3_camera_joint`         | `cgo3_horizontal_arm_link` -> `camera_link`            | `0 1 0`  | `-2.35619` 到 `0.7854 rad`    | `100` |   `-1` | `0.1` |

近似机械角度范围：

| 轴    |                   角度范围 |
| ----- | -------------------------: |
| yaw   |                 近似无限制 |
| roll  |  `-45 deg` 到 `45 deg` |
| pitch | `-135 deg` 到 `45 deg` |

## 云台关节控制器参数

云台使用 Gazebo `JointPositionController` 插件：

| 轴    | Gazebo topic             |       P |           I |         D |       指令限幅 |
| ----- | ------------------------ | ------: | ----------: | --------: | -------------: |
| roll  | `command/gimbal_roll`  | `0.8` |   `0.035` |  `0.02` | `-0.3 / 0.3` |
| pitch | `command/gimbal_pitch` | `0.8` | `0.01245` | `0.015` | `-0.3 / 0.3` |
| yaw   | `command/gimbal_yaw`   | `0.3` | `0.01245` | `0.015` | `-0.3 / 0.3` |

PX4 Gazebo bridge 最终向这些 Gazebo topic 发布关节位置指令：

```text
/model/x500_0/command/gimbal_roll
/model/x500_0/command/gimbal_pitch
/model/x500_0/command/gimbal_yaw
```

## 云台相机与云台 IMU 参数

云台相机：

| 参数（中文说明）   | 当前值                        |
| ----------------- | ----------------------------- |
| 传感器名          | `camera`                    |
| 类型              | `camera`                    |
| 坐标系            | `camera_link`               |
| 位姿              | `-0.0412 0 -0.162 0 0 3.14` |
| 图像格式          | `R8G8B8`                    |
| 图像尺寸          | `1280 x 720`                |
| 水平视场角        | `2.0 rad`                   |
| 裁剪距离 near/far | `0.05 / 15000`              |
| 更新率            | `30 Hz`                     |
| `always_on`（是否持续启用） | `1`                         |
| `visualize`（是否在 Gazebo 中可视化） | `true`                      |

云台相机 IMU：

| 参数（中文说明）                  | 当前值                        |
| -------------------------------- | ----------------------------- |
| 传感器名                         | `camera_imu`                |
| 类型                             | `imu`                       |
| 坐标系                           | `camera_link`               |
| 位姿                             | `-0.0412 0 -0.162 0 0 3.14` |
| 更新率                           | `250 Hz`                    |
| 陀螺 x/y/z 高斯噪声 `stddev`   | `0.00018665`                |
| 陀螺动态 bias `stddev`         | `3.8785e-05`                |
| 陀螺动态 bias 相关时间           | `1000`                      |
| 加速度 x/y/z 高斯噪声 `stddev` | `0.00186`                   |
| 加速度动态 bias `stddev`       | `0.006`                     |
| 加速度动态 bias 相关时间         | `300`                       |

## PX4 云台 Airframe 参数

主机 PX4 实例使用 `gz_x500_gimbal` airframe 启动。该 airframe 中的云台 mount 默认参数：

| PX4 参数（中文说明） |          当前值 | 含义                            |
| ------------------- | --------------: | ------------------------------- |
| `PX4_SIM_MODEL`（PX4 仿真模型名） | `x500_gimbal` | PX4 仿真模型名                  |
| `MNT_MODE_IN`（云台输入模式） |           `4` | MAVLink gimbal protocol v2 输入 |
| `MNT_MODE_OUT`（云台输出模式） |           `2` | MAVLink gimbal protocol v2 输出 |
| `MNT_RC_IN_MODE`（RC 云台输入模式） |           `1` | RC 输入使用角速度模式           |
| `MNT_MAN_ROLL`（手动 roll 通道） |           `1` | 手动/RC 模式下 AUX1 控制 roll   |
| `MNT_MAN_PITCH`（手动 pitch 通道） |           `2` | 手动/RC 模式下 AUX2 控制 pitch  |
| `MNT_MAN_YAW`（手动 yaw 通道） |           `3` | 手动/RC 模式下 AUX3 控制 yaw    |
| `MNT_RANGE_ROLL`（roll 输出范围） |     `180 deg` | roll 输出范围参数               |
| `MNT_RANGE_PITCH`（pitch 输出范围） |     `180 deg` | pitch 输出范围参数              |
| `MNT_RANGE_YAW`（yaw 输出范围） |     `720 deg` | yaw 输出范围参数                |

## 修改路径

根据要修改的对象选择对应路径：

| 修改目标                                                 | 修改路径                                                                                         |
| -------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| 主机模型 wrapper、风场开关、里程计插件                   | `px4_overlays/models/x500_gimbal_waypoint_wind/model.sdf`                                      |
| 目标机模型 wrapper、风场开关、里程计插件                 | `px4_overlays/models/x500_waypoint_wind/model.sdf`                                             |
| world 中加载哪两个模型、模型名、初始位姿                 | `px4_overlays/worlds/waypoint_tracking.sdf`                                                    |
| 运行时自动生成的 world                                   | `build/generated/worlds/waypoint_tracking.sdf`                                                 |
| PX4 实际运行时使用的 world 副本                          | `/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/waypoint_tracking.sdf`                      |
| 无人机机身质量、惯量、碰撞体、基础传感器、旋翼 link 位姿 | `/home/zk/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`                        |
| 无人机电机模型常数、电机方向、旋翼动力插件               | `/home/zk/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf`                             |
| 云台相对无人机的安装位姿、固定连接关节                   | `/home/zk/PX4-Autopilot/Tools/simulation/gz/models/x500_gimbal/model.sdf`                      |
| 云台质量、惯量、关节限位、相机、相机 IMU、云台 PID       | `/home/zk/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf`                           |
| PX4 airframe 云台 mount 默认参数                         | `/home/zk/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4019_gz_x500_gimbal`        |
| PX4 构建后的 airframe 副本                               | `/home/zk/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4019_gz_x500_gimbal` |
| ROS 云台视觉伺服调参，不是硬件参数                       | `src/uav_waypoint_tracking/config/gimbal_tracking.yaml`                                        |
| YOLO/ByteTrack 调参，不是硬件参数                        | `src/uav_waypoint_tracking/config/yolo_tracking.yaml`                                          |

不要把 `install/` 作为硬件参数修改位置；它是 `colcon` 生成目录。

不要把 `build/generated/worlds/waypoint_tracking.sdf` 作为源文件修改；它会被 `scripts/start_px4_gazebo.sh` 重新生成。

## 重启与重建说明

- 修改 `px4_overlays/` 下的 wrapper 或 world 后，重启 `scripts/start_px4_gazebo.sh` 即可重新生成并同步 world。
- 修改 `/home/zk/PX4-Autopilot/Tools/simulation/gz/models/...` 下的 PX4 官方 Gazebo 模型后，需要完整重启 PX4/Gazebo。
- 修改 PX4 `ROMFS` 下的 airframe 文件后，通常需要重新构建 PX4，确保 `build/px4_sitl_default/etc/init.d-posix/airframes/` 下的构建副本同步更新。
- 只修改 ROS YAML 调参文件时，通常只需要重启 ROS launch；当前工作区使用 `--symlink-install`，YAML-only 修改一般不需要重新 `colcon build`。
