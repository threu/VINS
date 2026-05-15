# simulation.launch 全流程代码解读

## 1. launch 文件整体解读

### 1.1 文件概览

`simulation.launch` 是 OpenVINS 的 **纯仿真模式** 启动文件。该模式不依赖任何外部传感器数据，而是通过 B-spline 轨迹生成模拟的 IMU 和相机数据，用于算法的可控评估和蒙特卡洛仿真。

文件路径: `src/open_vins/ov_msckf/launch/simulation.launch`

### 1.2 与其他模式的关键差异

| 特性 | subscribe | serial | simulation |
|------|-----------|--------|------------|
| 数据来源 | ROS 话题 | rosbag | 数学仿真 |
| 地面真值 | 可选外部文件 | 可选外部文件 | 精确可知 |
| 标定真值 | 未知 | 未知 | 精确可知 |
| 状态真值 | 未知 | 未知 | 精确可知 |
| 可复现性 | 取决于数据源 | 高（单线程） | 完全可复现 |
| 蒙特卡洛 | 不可行 | 困难 | 原生支持 |

### 1.3 仿真专用参数 (行 13-30)

```xml
<arg name="seed"        default="5" />       <!-- 随机种子 -->
<arg name="fej"         default="true" />     <!-- First Estimate Jacobians -->
<arg name="feat_rep"    default="GLOBAL_3D" /><!-- 特征参数化方式 -->
<arg name="num_clones"  default="11" />       <!-- 滑动窗口大小 -->
<arg name="num_slam"    default="50" />       <!-- SLAM特征数 -->
<arg name="num_pts"     default="100" />      <!-- 每帧特征点数 -->
<arg name="max_cameras" default="2" />        <!-- 相机数 -->
<arg name="use_stereo"  default="true" />     <!-- 双目 -->

<arg name="feat_dist_min" default="5.0" />   <!-- 最近特征距离 -->
<arg name="feat_dist_max" default="7.0" />   <!-- 最远特征距离 -->

<arg name="freq_cam"    default="10" />       <!-- 相机频率 (Hz) -->
<arg name="freq_imu"    default="400" />      <!-- IMU频率 (Hz) -->
```

**轨迹数据**:
```xml
<arg name="dataset" default="tum_corridor1_512_16_okvis.txt" />
<!-- 轨迹文件格式: time(s), pos(xyz), ori(xyzw) -->
```

**标定扰动参数**:
```xml
<arg name="sim_do_perturbation"         default="true" />
<arg name="sim_do_calibration"          default="true" />
<arg name="sim_do_calib_imu_intrinsics" default="true" />
<arg name="sim_do_calib_g_sensitivity"  default="true" />
```
当 `sim_do_perturbation=true` 时，估计器的初始标定值会被故意扰动偏离真值，用于测试标定估计算法。

**状态保存参数**:
```xml
<arg name="dosave_state"    default="true" />
<arg name="path_state_est"  default="$(find ov_eval)/data/sim/state_estimate.txt" />
<arg name="path_state_std"  default="$(find ov_eval)/data/sim/state_deviation.txt" />
<arg name="path_state_gt"   default="$(find ov_eval)/data/sim/state_groundtruth.txt" />
```

### 1.4 主节点参数 (行 48-97)

```xml
<node name="ov_msckf" pkg="ov_msckf" type="run_simulation" 
      output="screen" clear_params="true" required="true">
```

节点参数分为三类：

**仿真参数** (行 55-69):
| 参数 | 含义 |
|------|------|
| `sim_traj_path` | 仿真轨迹文件路径 |
| `sim_seed_state_init` | 特征地图生成种子（0=固定） |
| `sim_seed_measurements` | 测量噪声种子（可变，用于蒙特卡洛） |
| `sim_seed_preturb` | 标定扰动种子 |
| `sim_freq_cam` | 相机频率 |
| `sim_freq_imu` | IMU频率 |
| `sim_do_perturbation` | 是否扰动静态参数 |

**估计器参数** (行 75-96):
| 参数 | 含义 |
|------|------|
| `use_fej` | 是否使用 FEJ |
| `calib_cam_extrinsics` | 是否在线标定相机外参 |
| `calib_cam_intrinsics` | 是否在线标定相机内参 |
| `calib_cam_timeoffset` | 是否在线标定相机-IMU时间偏移 |
| `calib_imu_intrinsics` | 是否在线标定IMU内参 |
| `calib_imu_g_sensitivity` | 是否在线标定g-sensitivity |
| `max_clones` | 滑动窗口最大克隆数 |
| `max_slam` | 最大SLAM特征数 |
| `feat_rep_msckf/slam/aruco` | 特征表示 (GLOBAL_3D, ANCHORED_MSCKF, etc.) |
| `num_pts` | 每帧提取的特征点数 |

**轨迹保存节点** (行 101-112):
```xml
<group if="$(arg dosave_pose)">
    <!-- 保存估计轨迹 -->
    <node name="recorder_estimate" pkg="ov_eval" type="pose_to_file">
        <param name="topic"  value="/ov_msckf/poseimu" />
        <param name="output" value="$(arg path_est)" />
    </node>
    <!-- 保存地面真值轨迹 -->
    <node name="recorder_groundtruth" pkg="ov_eval" type="pose_to_file">
        <param name="topic"  value="/ov_msckf/posegt" />
        <param name="output" value="$(arg path_gt)" />
    </node>
</group>
```

---

## 2. run_simulation.cpp 全流程解读

文件路径: `src/open_vins/ov_msckf/src/run_simulation.cpp`

### 2.1 全局变量 (行 42-48)

```cpp
std::shared_ptr<Simulator> sim;        // 仿真器 (生成模拟数据)
std::shared_ptr<VioManager> sys;       // VIO核心管理器
std::shared_ptr<ROS1Visualizer> viz;   // ROS1可视化器 (带sim引用)
```

与 subscribe/serial 模式不同，这里多了一个 `Simulator` 对象，且可视化器持有仿真器的引用。

### 2.2 main() 函数总流程

```
main()
  ├── [阶段1: 初始化] (行 54-108)
  │   ├── ROS 初始化
  │   ├── 创建 YamlParser, 加载配置
  │   ├── 创建 VioManagerOptions
  │   │   ├── params.print_and_load(parser)        // 加载估计器参数
  │   │   └── params.print_and_load_simulation(parser)  // 加载仿真参数 (独有!)
  │   │
  │   ├── params.num_opencv_threads = 0    // 单线程 (可复现)
  │   ├── params.use_multi_threading_pubs = false  // 同步发布
  │   ├── params.use_multi_threading_subs = false  // 同步订阅
  │   │
  │   ├── sim = make_shared<Simulator>(params)     // 创建仿真器
  │   ├── sys = make_shared<VioManager>(params)    // 创建VIO管理器
  │   └── viz = make_shared<ROS1Visualizer>(nh, sys, sim)  // 可视化器(带sim)
  │
  ├── [阶段2: 地面真值初始化] (行 115-131)
  │   ├── sim->get_state(next_imu_time, imustate)  // 获取初始真实状态
  │   ├── 减去相机-IMU时间偏移
  │   └── sys->initialize_with_gt(imustate)        // 用真值初始化
  │
  └── [阶段3: 主循环] (行 138-178)
      └── while (sim->ok() && ros::ok()):
            ├── sim->get_next_imu(...)  → sys->feed_measurement_imu()
            │                           → viz->visualize_odometry()
            │
            └── sim->get_next_cam(...)  → sys->feed_measurement_simulation()
                                        → viz->visualize()
```

### 2.3 仿真主循环详解

```cpp
// 相机缓冲 (用于保证处理顺序)
double buffer_timecam = -1;
std::vector<int> buffer_camids;
std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> buffer_feats;

while (sim->ok() && ros::ok()) {

    // ===== IMU 处理 =====
    ov_core::ImuData message_imu;
    bool hasimu = sim->get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
    if (hasimu) {
        sys->feed_measurement_imu(message_imu);    // 喂入VIO系统
        viz->visualize_odometry(message_imu.timestamp);  // 高频里程计
    }

    // ===== 相机处理 =====
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    bool hascam = sim->get_next_cam(time_cam, camids, feats);
    if (hascam) {
        // 关键: 先处理上一帧，再缓存当前帧
        // 这保证了相机数据处理的正确时间顺序
        if (buffer_timecam != -1) {
            sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
            viz->visualize();
        }
        buffer_timecam = time_cam;
        buffer_camids = camids;
        buffer_feats = feats;
    }
}
```

**缓冲设计**: 相机数据延迟一帧处理，确保在处理第k帧时，第k+1帧的信息已可用于特征管理（如判断哪些特征在下一帧丢失了）。

---

## 3. Simulator 仿真器深度解析

### 3.1 Simulator 构造函数流程

```
Simulator::Simulator(params)
  ├── 1. 加载轨迹文件 (time, pos, ori 格式)
  ├── 2. B-spline 拟合轨迹
  │     - 使用 BsplineSE3 类对离散位姿进行 B-spline 插值
  │     - 可获取任意时刻的连续位姿、速度、加速度
  │
  ├── 3. 生成3D特征点地图
  │     - 在轨迹起始位置前方的锥形区域内随机生成特征点
  │     - 特征距离: [sim_min_feature_gen_distance, sim_max_feature_gen_distance]
  │     - 使用 sim_seed_state_init 作为随机种子
  │
  ├── 4. 生成IMU测量序列
  │     - 按 sim_freq_imu 频率采样
  │     - 从 B-spline 获取真实角速度和加速度
  │     - 添加噪声: n_w ~ N(0, sigma_w²), n_a ~ N(0, sigma_a²)
  │     - 噪声种子: sim_seed_measurements
  │
  ├── 5. 生成相机测量序列
  │     - 按 sim_freq_cam 频率采样
  │     - 将3D特征点投影到各相机平面
  │     - 添加像素噪声: n_uv ~ N(0, sigma_px²)
  │     - 仅保留在当前视野内的特征点
  │
  └── 6. (可选) 扰动初始标定参数
        - 相机内参: 添加随机扰动
        - 相机外参: 添加旋转和平移扰动
        - IMU内参: 添加缩放和轴偏差扰动
        - 时间偏移: 添加随机偏移
        - 扰动种子: sim_seed_preturb
```

### 3.2 IMU 测量仿真

```
真实IMU测量生成:
  给定: B-spline轨迹 (可获取任意时刻的位姿/速度/加速度/角速度)

  1. 获取真实角速度 (在世界系):
     ω_true = B-spline 角速度

  2. 获取真实加速度 (在世界系):
     a_true = B-spline 加速度

  3. 旋转到IMU体坐标系:
     ω_I = R_ItoG^T * ω_true
     a_I = R_ItoG^T * (a_true - g)  // 减去重力

  4. 添加噪声:
     wm = ω_I + n_g   (n_g ~ N(0, sigma_w²))
     am = a_I + n_a   (n_a ~ N(0, sigma_a²))
```

### 3.3 相机测量仿真

```
相机测量生成:
  对于每个3D特征点 p_FinG:

  1. 转换到相机坐标系:
     p_FinC = R_ItoC * R_GtoI * (p_FinG - p_IinG) + p_IinC

  2. 透视投影:
     u = fx * (p_FinC.x / p_FinC.z) + cx
     v = fy * (p_FinC.y / p_FinC.z) + cy

  3. 畸变 (radtan/equidistant):
     ud, vd = distort(u, v, k1, k2, p1, p2)

  4. 添加像素噪声:
     um = ud + n_u  (n_u ~ N(0, sigma_px²))
     vm = vd + n_v  (n_v ~ N(0, sigma_px²))

  5. 检查是否在图像范围内:
     0 <= um < width && 0 <= vm < height
```

### 3.4 标定扰动

当 `sim_do_perturbation=true` 时，估计器初始参数被有意偏离真值：

```
扰动策略:
  - 相机内参 fx,fy:    N(0, 0.1*fx) 噪声
  - 相机内参 cx,cy:    N(0, 50) 像素噪声
  - 相机畸变:          N(0, 0.01) 噪声
  - 相机外参旋转:      随机旋转轴, 角度 N(0, 3°)
  - 相机外参平移:      N(0, 0.02m) 噪声
  - IMU内参缩放:       N(0, 0.01) 噪声
  - 时间偏移:          N(0, 0.02s) 噪声
```

---

## 4. feed_measurement_simulation() 深度解析

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp` (行 191-254)

```cpp
void VioManager::feed_measurement_simulation(timestamp, camids, feats)
```

```
feed_measurement_simulation()
  ├── 1. 检查/创建 TrackSIM 追踪器:
  │     - 首次调用时将 trackFEATS 转为 TrackSIM
  │     - 同步替换 initializer 和 zupt 中的追踪器引用
  │
  ├── 2. 喂入仿真特征:
  │     trackSIM->feed_measurement_simulation(timestamp, camids, feats)
  │     - 直接将仿真投影特征作为"被追踪"特征
  │     - 跳过实际的特征提取和匹配过程
  │
  ├── 3. (可选) 零速更新:
  │     updaterZUPT->try_update(state, timestamp)
  │
  ├── 4. 检查 VIO 是否已初始化:
  │     if (!is_initialized_vio) → 错误退出
  │     (仿真模式要求地面真值预初始化)
  │
  └── 5. 核心更新:
        do_feature_propagate_update(message)
        - 与 subscribe/serial 模式使用完全相同的更新函数
```

**TrackSIM 的关键作用**: 跳过真实图像处理（特征提取、KLT追踪、描述子匹配），直接将仿真生成的特征匹配对作为输入。这样：
- 消除了前端追踪误差对评估的影响
- 可以精确控制特征数量和质量
- 运行速度极快（无图像处理）

---

## 5. 仿真模式独有的评估能力

### 5.1 全状态真值比较

仿真模式可以获取滤波器的**完整内部状态真值**，不仅限于位姿：

```cpp
// 状态估计 vs 真值
Eigen::Matrix<double, 17, 1> state_est = state->_imu->value();
Eigen::Matrix<double, 17, 1> state_gt;
sim->get_state(timestamp, state_gt);

// 可评估的误差:
// - 位姿误差 (q, p)
// - 速度误差 (v)
// - 陀螺仪偏置误差 (bg)
// - 加速度计偏置误差 (ba)
// - 相机内参误差
// - 相机外参误差
// - IMU内参误差 (Dw, Da, Tg)
// - 重力敏感性误差
// - 相机-IMU时间偏移误差
```

### 5.2 NEES (Normalized Estimation Error Squared)

仿真模式是计算 NEES 的理想场景：

```
NEES = (x - x_gt)^T · P^{-1} · (x - x_gt)

理论值: E[NEES] = dim(x)  (对N自由度卡方分布)
- 位姿NEES: 期望 ≈ 6
- 完整状态NEES: 期望 ≈ 完整状态维度

NEES > 期望 → 滤波器过于自信 (协方差低估)
NEES < 期望 → 滤波器过于保守 (协方差高估)
```

### 5.3 蒙特卡洛仿真

通过改变 `sim_seed_measurements` 运行多次：
- 相同真值轨迹 + 相同特征地图
- 不同的测量噪声实现
- 统计 RMSE/NEES 的均值和方差

---

## 6. 完整数据流总结

```
┌─────────────────────────────────────────────────────────┐
│              simulation.launch 启动                      │
│  roslaunch ov_msckf simulation.launch seed:=5           │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│              run_simulation.cpp main()                   │
│                                                         │
│  1. 创建 Simulator(params)                              │
│     ├─ 加载轨迹 → B-spline 拟合                         │
│     ├─ 生成特征地图                                     │
│     ├─ 生成 IMU 测量序列 (400Hz)                        │
│     ├─ 生成相机测量序列 (10Hz)                          │
│     └─ 扰动初始标定                                     │
│                                                         │
│  2. 创建 VioManager(params)                             │
│                                                         │
│  3. 真值初始化:                                          │
│     sim->get_state() → sys->initialize_with_gt()        │
│                                                         │
│  4. 主循环:                                              │
│     while sim->ok():                                    │
│       ├─ sim->get_next_imu()                            │
│       │   └─ sys->feed_measurement_imu()                │
│       │       ├─ propagator->feed_imu()                 │
│       │       └─ (已初始化, 跳过initializer)              │
│       │                                                 │
│       └─ sim->get_next_cam()                            │
│           └─ sys->feed_measurement_simulation()          │
│               ├─ trackSIM->feed_measurement_simulation() │
│               └─ do_feature_propagate_update()           │
│                    ├─ propagate_and_clone()              │
│                    ├─ updaterMSCKF->update()             │
│                    ├─ updaterSLAM->update()              │
│                    ├─ updaterSLAM->delayed_init()        │
│                    └─ marginalize_old_clone()            │
│                                                         │
│  5. visualize_final() → 输出完整评估结果                  │
└─────────────────────────────────────────────────────────┘
```

## 7. 仿真模式的特点与适用场景

### 7.1 核心优势
- **完全可控**: 所有参数（噪声水平、特征密度、轨迹复杂度）可控
- **真值完整**: 不仅可以评估位姿，还能评估速度、偏置、标定参数
- **可复现**: 固定种子产生完全相同的仿真数据
- **蒙特卡洛友好**: 可以高效运行数百次试验
- **无前端误差**: TrackSIM 跳过真实图像处理

### 7.2 典型用途
- **算法开发**: 在无传感器噪声的理想条件下验证算法正确性
- **一致性检验**: 通过 NEES 验证协方差估计的一致性
- **灵敏度分析**: 研究不同噪声水平、标定误差对性能的影响
- **回归测试**: 确保代码修改不会破坏估计精度
