# SAM6 — Surface-to-Air Missile 6 DoF Simulation

SAM6 是一个地对空导弹六自由度 (6 DoF) 仿真程序，基于 CADAC++ 架构，使用 C++ 编写。

原作者 Peter H. Zipfel (MaSTech)，Release 3.0 (2023-10-27)。
原始工程为 Windows / MSVC，已移植到 Linux / GCC。

## 功能

- 最多 3 枚导弹同时攻击最多 3 个目标（火箭或飞机）
- 目标先发射，导弹由跟踪雷达延迟发射
- 中段指令制导（line guidance），末段比例导引（pro-nav）
- RF 或 IR 导引头，带高斯偏差和马尔可夫闪烁噪声
- 蒙特卡洛仿真能力
- 模块化架构，事件调度

## 目录结构

```
SAM6_250217/
├── README.md
└── SAM6/
    ├── Makefile              # 构建配置
    ├── input.asc             # 当前活动场景配置
    ├── SAM6                  # 编译产物（可执行文件）
    ├── src/                  # 源代码
    │   ├── *.cpp (28 个)     # 模块实现
    │   └── *.hpp (4 个)      # 头文件
    ├── scenarios/            # 场景配置文件
    │   ├── input.asc                   # 场景副本
    │   ├── input_SAM_RF_AC_Radar_*.asc # RF 导弹 vs 飞机
    │   ├── input_SAM_IR_SRBM_Radar_*.asc # IR 导弹 vs 弹道导弹
    │   ├── input_SAM_Ballistic.asc     # 弹道场景
    │   └── input_SAM_autopilot.asc     # 自动驾驶仪测试
    ├── data/                 # 气动/推进/弹道数据表
    │   ├── SAM_aero_deck.asc
    │   ├── SAM_prop_deck.asc
    │   ├── SAM_traj_deck.asc
    │   ├── SRBM_aero_deck.asc
    │   └── SRBM_traj_deck_ballistic.asc
    ├── doc/                  # 原始文档
    │   ├── readme.asc
    │   └── documentation.asc
    └── build/                # 构建中间产物（.o 文件）
```

## 类层次

```
Cadac (抽象基类)
├── Flat6 (6 DoF: 环境 + 运动学 + 欧拉 + 牛顿)
│   └── Missile  — 完整 6 DoF 导弹
├── Flat3 (3 DoF: 环境 + 运动学 + 牛顿)
│   ├── Rocket   — 5 DoF 火箭目标
│   └── Aircraft — 3 DoF 飞机目标
└── Flat0 (0 DoF: 运动学 + 牛顿)
    └── Radar    — 0 DoF 跟踪雷达
```

## 编译

```bash
cd SAM6
make
```

需要 g++ 和 make（Ubuntu 下 `sudo apt install build-essential`）。

编译产物为当前目录下的 `SAM6` 可执行文件，中间文件在 `build/` 目录。

清理：

```bash
make clean
```

## 运行

```bash
./SAM6
```

程序读取当前目录下的 `input.asc` 作为输入配置，运行结束后生成以下输出文件：

| 文件 | 内容 |
|------|------|
| `plot1.asc` ~ `plotN.asc` | 各导弹的逐时间步数据 |
| `plot.asc` | 合并后的所有导弹数据 |
| `traj.asc` | 通信总线 (combus) 数据，包含所有飞行器 |
| `doc.asc` | 模块变量文档 |
| `tabout.asc` | 屏幕输出的文件副本（需在 input.asc 中启用 `y_tabout`） |

输出文件可用 MATLAB / Python / Excel 绘图。启用 CSV 格式：在 `input.asc` 的 OPTIONS 行加入 `y_csv`，会额外生成 `plot.csv` 和 `traj.csv`。

## 切换场景

将 `scenarios/` 下的场景文件复制为 `input.asc`：

```bash
cp scenarios/input_SAM_IR_SRBM_Radar_#1.asc input.asc
./SAM6
```

## 输入配置 (`input.asc`)

`input.asc` 是主配置文件，格式规则：

- 大写单词为关键字
- 每行数据以关键字开头
- 数据块以关键字开始、`END` 结束
- `//` 开头为注释
- 不使用 `=` 号赋值

主要区段：

```
TITLE         — 标题
MONTE         — 蒙特卡洛参数（运行次数, 随机种子）
OPTIONS       — 输出控制开关
MODULES       — 模块调用顺序
TIMING        — 时间步长
VEHICLES      — 飞行器定义（MISSILE6, AIRCRAFT3, RADAR0 等）
ENDTIME       — 仿真终止时间
STOP          — 文件结束标记
```

## 内置场景

| 文件 | 描述 |
|------|------|
| `input_SAM_RF_AC_Radar_#1_#2_#3.asc` | 3 枚 RF 导弹 vs 3 架飞机，雷达跟踪 |
| `input_SAM_RF_AC_Radar_#1.asc` | 单枚 RF 导弹 vs 单架飞机 |
| `input_SAM_IR_SRBM_Radar_#1.asc` | IR 导弹 vs 弹道导弹 |
| `input_SAM_IR_SRBM_Radar_#1_#2_#3.asc` | 3 枚 IR 导弹 vs 3 枚弹道导弹 |
| `input_SAM_Ballistic.asc` | 弹道场景 |
| `input_SAM_autopilot.asc` | 自动驾驶仪测试 |

## 导弹模块调用链

```
environment → kinematics → propulsion → aerodynamics → ins → sensor
→ guidance → control → actuator → forces → euler → newton → intercept
```

## 源文件说明

| 文件 | 功能 |
|------|------|
| `execution.cpp` | 主函数 `main()` 和仿真执行循环 |
| `class_hierarchy.hpp` | 类层次定义（Cadac, Flat6, Missile, ...） |
| `class_functions.cpp` | 类构造/析构函数 |
| `global_header.hpp` | 全局结构和类（Module, Variable, Event, Packet, Table, Datadeck） |
| `global_constants.hpp` | 全局常量（物理常数、数组尺寸） |
| `global_functions.cpp` | 全局函数（输入解析、输出合并、CSV 生成） |
| `utility_header.hpp` | Matrix 类和工具函数声明 |
| `utility_functions.cpp` | Matrix 类实现、随机数、大气模型、数值积分 |
| `kinematics.cpp` | 运动学模块 |
| `newton.cpp` | 牛顿动力学模块 |
| `euler.cpp` | 欧拉方程模块 |
| `environment.cpp` | 大气和重力环境 |
| `aerodynamics.cpp` | 气动力/力矩（导弹） |
| `propulsion.cpp` | 推力模型 |
| `forces.cpp` | 力的合成 |
| `actuator.cpp` | 舵机模型（二阶动力学） |
| `control.cpp` | 自动驾驶仪（滚转/速率/加速度控制） |
| `guidance.cpp` | 制导律（中段线导、末段比例导引） |
| `sensor.cpp` | 导引头模型（RF/IR）和雷达跟踪 |
| `ins.cpp` | 惯性导航系统 |
| `intercept.cpp` | 拦截判定 |
| `tvc.cpp` | 推力矢量控制 |
| `rcs.cpp` | 反作用控制系统 |
| `missile_functions.cpp` | 导弹特有函数 |
| `aircraft_functions.cpp` | 飞机目标函数 |
| `aircraft_modules.cpp` | 飞机目标模块 |
| `rocket_functions.cpp` | 火箭目标函数 |
| `rocket_modules.cpp` | 火箭目标模块 |
| `radar_functions.cpp` | 雷达函数 |
| `radar_modules.cpp` | 雷达模块 |
| `flat0_modules.cpp` | Flat0 基类模块 |
| `flat3_modules.cpp` | Flat3 基类模块 |

## 参考文献

- Peter H. Zipfel, "Modeling and Simulation of Aerospace Vehicle Dynamics", AIAA, 3rd Edition 2014
- Peter H. Zipfel, "Missile and Rocket Simulation Workshop in Four Days", Amazon, 2nd Edition 2024
- UDEMY: "Missile and Rocket Simulations in C++"
