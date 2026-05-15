# OpenVINS 代码深度解读文档

本文件夹包含 OpenVINS 工程的完整代码解读文档，涵盖了三种启动模式（subscribe、serial、simulation）和轨迹评估工具的全流程分析，以及对应的数学公式和计算原理。

## 文档索引

| 序号 | 文件 | 内容 |
|------|------|------|
| 01 | [subscribe_launch_analysis.md](01_subscribe_launch_analysis.md) | subscribe.launch 全流程代码解读 |
| 02 | [serial_launch_analysis.md](02_serial_launch_analysis.md) | serial.launch 全流程代码解读 |
| 03 | [simulation_launch_analysis.md](03_simulation_launch_analysis.md) | simulation.launch 全流程代码解读 |
| 04 | [trajectory_evaluation_analysis.md](04_trajectory_evaluation_analysis.md) | 轨迹评估 (ov_eval) 全流程代码解读 |
| 05 | [formula_and_principles.md](05_formula_and_principles.md) | 全流程公式计算与计算原理 |

## 阅读建议

1. **快速了解系统**: 从 `01_subscribe_launch_analysis.md` 开始，这是最常用的 ROS 订阅模式
2. **理解核心算法**: 阅读 `05_formula_and_principles.md` 获取完整的数学推导
3. **离线评测**: 参考 `02_serial_launch_analysis.md` 了解串行 rosbag 处理模式
4. **算法验证**: 参考 `03_simulation_launch_analysis.md` 了解仿真模式（支持蒙特卡洛评估）
5. **结果评估**: 阅读 `04_trajectory_evaluation_analysis.md` 了解 ATE/RPE/NEES 计算

## 源代码根目录

所有代码位于: `src/open_vins/`

```
src/open_vins/
├── ov_msckf/          # 核心 MSCKF VIO 估计器
│   ├── launch/        # ROS launch 文件
│   ├── src/
│   │   ├── core/      # VioManager (核心调度)
│   │   ├── state/     # State, Propagator (状态管理与IMU传播)
│   │   ├── update/    # UpdaterMSCKF, UpdaterSLAM (EKF更新)
│   │   ├── ros/       # ROS1Visualizer, ROS2Visualizer
│   │   ├── sim/       # Simulator (仿真器)
│   │   └── utils/     # 工具类
│   └── config/        # 配置文件
├── ov_core/           # 核心库 (相机模型, 特征追踪, 初始化)
├── ov_init/           # 惯性初始化器
├── ov_eval/           # 轨迹评估工具
└── ov_data/           # 数据集/轨迹/配置文件
```

## 主要参考

- OpenVINS 官网文档: https://docs.openvins.com
- Geneva et al., "OpenVINS: An Open Platform for Visual-Inertial Research", ICRA 2020
- Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation", ICRA 2007
