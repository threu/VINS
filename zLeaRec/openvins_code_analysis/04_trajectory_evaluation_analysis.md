# 轨迹评估 (ov_eval) 全流程代码解读

## 1. 评估工具概览

`ov_eval` 包提供了完整的轨迹评估工具链，位于 `src/open_vins/ov_eval/`。评估遵循 Zhang & Scaramuzza 2018 IROS 论文的标准方法。

### 1.1 工具清单

| 可执行文件 | 源文件 | 功能 |
|-----------|--------|------|
| `error_singlerun` | `error_singlerun.cpp` | 单次运行的完整误差分析 |
| `error_dataset` | `error_dataset.cpp` | 多算法/多数据集批量比较 |
| `error_simulation` | `error_simulation.cpp` | 仿真模式专用评估 |
| `error_comparison` | `error_comparison.cpp` | 多次运行的统计比较 |
| `plot_trajectories` | `plot_trajectories.cpp` | 轨迹可视化 |
| `pose_to_file` | `pose_to_file.cpp` | ROS 话题记录到文件 |
| `live_align_trajectory` | `live_align_trajectory.cpp` | 实时轨迹对齐 |
| `timing_flamegraph` | `timing_flamegraph.cpp` | 时间性能火焰图 |
| `timing_histogram` | `timing_histogram.cpp` | 时间性能直方图 |
| `format_converter` | `format_converter.cpp` | 数据格式转换 |

### 1.2 核心数据结构

#### 位姿文件格式

估计文件和地面真值文件使用统一格式：
```
# timestamp tx ty tz qx qy qz qw cov1 cov2 ...
timestamp_in_seconds pos_x pos_y pos_z quat_x quat_y quat_z quat_w [covariances...]
```

#### Statistics 统计结构

文件路径: `src/open_vins/ov_eval/src/utils/Statistics.h`

```cpp
struct Statistics {
    std::vector<double> timestamps;    // 时间戳
    std::vector<double> values;        // 误差值
    std::vector<double> values_bound;  // 3-sigma 边界值 (如有协方差)
    
    double min, max, mean, median, std, rmse;
    
    void calculate();  // 计算所有统计量
    void clear();      // 清空数据
};
```

---

## 2. error_singlerun — 单次运行误差分析

### 2.1 命令行用法

```bash
./error_singlerun <align_mode> <file_gt.txt> <file_est.txt>
# 例如:
./error_singlerun posyaw gt.txt traj_estimate.txt
```

对齐模式: `posyaw` (位置+yaw), `se3` (完整SE3), `sim3` (带尺度SE3), `none`

### 2.2 主流程

```
error_singlerun main()
  ├── 1. 加载文件:
  │     Loader::load_data(path_gt, times, poses, cov_ori, cov_pos)
  │
  ├── 2. 创建 ResultTrajectory 对象:
  │     ResultTrajectory traj(path_est, path_gt, align_mode)
  │     (内部完成: 加载→时间关联→轨迹对齐)
  │
  ├── 3. 计算 ATE (Absolute Trajectory Error):
  │     traj.calculate_ate(error_ori, error_pos)
  │     → 输出: rmse, mean, min, max, std
  │
  ├── 4. 计算 RPE (Relative Pose Error):
  │     segments = [8.0, 16.0, 24.0, 32.0, 40.0] 米
  │     traj.calculate_rpe(segments, error_rpe)
  │     → 对每个段长输出: median_ori, median_pos
  │
  ├── 5. 计算 NEES (如果数据包含协方差):
  │     traj.calculate_nees(nees_ori, nees_pos)
  │
  └── 6. 计算逐点误差:
        traj.calculate_error(posx,posy,posz, orix,oriy,oriz, roll,pitch,yaw)
        → 输出每个自由度的误差时序
```

---

## 3. ResultTrajectory 核心类深度解析

文件路径: `src/open_vins/ov_eval/src/calc/ResultTrajectory.cpp`

### 3.1 构造函数 — 数据加载与对齐

```cpp
ResultTrajectory::ResultTrajectory(path_est, path_gt, alignment_method)
```

```
构造函数流程:
  ├── 1. 加载估计轨迹: Loader::load_data(path_est, est_times, est_poses, est_covori, est_covpos)
  │     - est_poses[i]: [px, py, pz, qx, qy, qz, qw]  (7维)
  │     - est_covori[i]: 3x3 姿态协方差
  │     - est_covpos[i]: 3x3 位置协方差
  │
  ├── 2. 加载地面真值: Loader::load_data(path_gt, gt_times, gt_poses, ...)
  │
  ├── 3. 轨迹长度检查: 估计/真值长度比应在 [0.9, 1.1] 范围内
  │
  ├── 4. 时间关联: AlignUtils::perform_association(offset=0, max_diff=0.02s, ...)
  │     - 找到时间戳匹配 (差值 < 0.02s) 的位姿对
  │     - 确保估计和真值在相同时间戳上有对应位姿
  │
  └── 5. 轨迹对齐:
        AlignTrajectory::align_trajectory(est_poses, gt_poses, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, method)
        AlignTrajectory::align_trajectory(gt_poses, est_poses, R_GTtoEST, t_GTinEST, s_GTtoEST, method)
        
        生成对齐后的轨迹:
          est_poses_alignedtoGT[i] = [s*R*est_pos + t, quat_multiply(est_quat, Inv(q_align))]
```

### 3.2 时间关联 (perform_association)

```cpp
AlignUtils::perform_association(offset, max_difference, est_times, gt_times, est_poses, gt_poses)
```

```
时间关联算法:
  对于 est_times[tid] 的每个估计位姿:
    在 gt_times 中寻找最近的匹配:
      min_diff = inf
      for each gt_times[tid2]:
        diff = |est_times[tid] + offset - gt_times[tid2]|
        if diff < min_diff: min_diff = diff, best_idx = tid2
    
    如果 min_diff < max_difference (默认 0.02s):
      保留该匹配
    否则:
      丢弃该估计位姿

  结果: 两组轨迹具有完全相同数量的时刻
```

### 3.3 轨迹对齐 (align_trajectory)

支持的四种对齐模式：

#### posyaw (位置+偏航角对齐)
```
假设重力方向在估计和真值坐标系中已对齐
仅估计: 4DoF 变换 (偏航角 + 3D平移)

算法:
  1. 计算每个轨迹的均值位置: p̄_est, p̄_gt
  2. 去均值: p'_est = p_est - p̄_est, p'_gt = p_gt - p̄_gt
  3. 构建 x-y 平面相关矩阵: C = Σ p'_gt[:2] · p'_est[:2]^T
  4. 最优偏航角: yaw = atan2(C(0,1)-C(1,0), C(0,0)+C(1,1))
  5. 旋转矩阵: R = Rz(yaw)
  6. 平移: t = p̄_gt - R * p̄_est
```

#### se3 (完整SE(3)对齐)
```
使用 Umeyama 算法 (1991 TPAMI):
  给定两组对应3D点，求解最优旋转 R 和平移 t

算法:
  1. 计算均值: p̄_est, p̄_gt
  2. 去均值: p'_est, p'_gt
  3. 协方差矩阵: Σ = p'_gt · p'_est^T / N
  4. SVD: Σ = U · D · V^T
  5. 旋转: R = U · S · V^T  (S = diag(1, 1, det(U·V^T))
  6. 平移: t = p̄_gt - R * p̄_est
```

#### sim3 (带尺度的相似变换)
```
扩展 Umeyama 算法 (含尺度):
  1-4. 同 se3
  5. 尺度: s = tr(D · S) / tr(p'_est · p'_est^T) / N
  6. 旋转: R = U · S · V^T
  7. 平移: t = p̄_gt - s · R · p̄_est
```

#### none (不对齐)
```
直接使用原始坐标系中的轨迹 (假设已在同一坐标系)
```

---

## 4. ATE — 绝对轨迹误差

### 4.1 calculate_ate()

```cpp
void ResultTrajectory::calculate_ate(Statistics &error_ori, Statistics &error_pos)
```

```
ATE 计算 (逐时间戳):
  对于 i = 0..N-1:
    
    1. 姿态误差:
       e_R = R_est_aligned^T * R_gt
       ori_err = (180/π) * ||log_so3(e_R)||   (度)
    
    2. 位置误差:
       pos_err = ||p_gt - p_est_aligned||    (米)
    
    3. 记录:
       error_ori.values[i] = ori_err
       error_pos.values[i] = pos_err

统计量计算 (Statistics::calculate):
  mean  = Σ values[i] / N
  rmse  = sqrt(Σ values[i]² / N)
  std   = sqrt(Σ (values[i] - mean)² / N)
  min   = min(values)
  max   = max(values)
  median = 排序后的中间值
```

### 4.2 calculate_ate_2d() — 2D ATE

仅考虑 x-y 平面和偏航角：
```
姿态误差 (仅偏航角):
  e_R = R_est_aligned^T * R_gt
  ori_err = (180/π) * |log_so3(e_R)(2)|  // 仅z分量

位置误差 (仅x-y):
  pos_err = ||p_gt[:2] - p_est_aligned[:2]||
```

---

## 5. RPE — 相对位姿误差

### 5.1 calculate_rpe()

```cpp
void ResultTrajectory::calculate_rpe(segment_lengths, error_rpe)
```

RPE 评估**局部漂移**，通过比较轨迹上固定距离段内的相对位姿变化。

```
RPE 计算 (对每个段长 d):
  
  1. 为每个起始索引 i 找到结束索引 j:
     使得 accum_dist[j] ≈ accum_dist[i] + d
  
  2. 对每个有效段:
     
     a) 估计的相对变换:
        T_est_start → T_est_end
        T_rel_est = T_est_start^{-1} * T_est_end
    
     b) 真值的相对变换:
        T_gt_start → T_gt_end
        T_rel_gt = T_gt_start^{-1} * T_gt_end
    
     c) 误差变换:
        T_error = Inv(T_rel_gt) * T_rel_est
    
     d) 旋转到世界坐标系:
        T_error_in_w = T_est_end_rot * T_error * T_est_end_rot^T
    
     e) 误差:
        pos_err = ||T_error_in_w.translation||
        ori_err = (180/π) * ||log_so3(T_error_in_w.rotation)||

  3. 统计每个段长的所有误差
```

**关键理解**: RPE 计算旋转误差时将其旋转到世界坐标系，这样偏航角误差（通常是视觉惯性里程计中最大的误差源）能被正确评估。

### 5.2 段长选择

```
compute_comparison_indices_length(distances, distance, max_dist_diff):

  对每个起始索引 i:
    目标距离 = distances[i] + distance
    
    查找最近的地面真值点:
      best_idx = -1
      for j = i to N-1:
        if |distances[j] - 目标距离| < best_error:
          best_idx = j
          best_error = |distances[j] - 目标距离|
    
    如果 best_error < max_dist_diff → valid segment
    否则 → invalid (-1)
```

---

## 6. NEES — 归一化估计误差平方

### 6.1 calculate_nees()

NEES 评估滤波器的**一致性**（即协方差是否真实反映估计精度）。

```cpp
void ResultTrajectory::calculate_nees(nees_ori, nees_pos)
```

前提条件: 轨迹数据必须包含协方差信息（使用 PoseWithCovarianceStamped 或 Odometry 记录）

```
NEES 计算:

  1. 姿态 NEES:
     e_R = R_gt_aligned * R_est^T
     error_i = -log_so3(e_R)  (3D误差向量)
     nees_ori = error_i^T * cov_ori^{-1} * error_i
  
  2. 位置 NEES:
     err_pos = p_gt_aligned - p_est
     nees_pos = err_pos^T * cov_pos^{-1} * err_pos
```

**NEES 理论性质**:
- 如果滤波器一致且误差服从高斯分布: `NEES ~ χ²(n)` (n=3 自由度)
- `E[NEES_ori] = E[NEES_pos] = 3`
- NEES > 3: 滤波器**过于自信**（协方差低估了真实误差）
- NEES < 3: 滤波器**过于保守**（协方差高估了真实误差）

### 6.2 与 ROS1Visualizer 中实时 NEES 计算的对比

ROS1Visualizer 中的实时 NEES 计算 (`publish_groundtruth()`):

```cpp
// 姿态 NEES (实时版本)
Eigen::Vector3d quat_diff_vec = quat_diff.block(0, 0, 3, 1);
double ori_nees = 2 * quat_diff_vec.dot(cov_ori.inverse() * 2 * quat_diff_vec);

// 位置 NEES
double pos_nees = err_pos.transpose() * cov_pos.inverse() * err_pos;
```

这与 ResultTrajectory 中使用 `-log_so3(e_R)` 的方法等价（对于小误差）。

---

## 7. error_dataset — 多算法批量比较

### 7.1 目录结构约定

```
<folder_algorithms>/
  ├── algorithm1/
  │   ├── dataset1/
  │   │   ├── run1.txt
  │   │   ├── run2.txt
  │   │   └── run3.txt
  │   └── dataset2/
  │       └── ...
  └── algorithm2/
      └── ...
```

### 7.2 主流程

```
error_dataset main():
  ├── 1. 搜索所有算法目录
  ├── 2. 对每个算法:
  │     ├── 查找对应数据集的运行文件
  │     ├── 对每次运行:
  │     │   ├── 创建 ResultTrajectory
  │     │   ├── 计算 ATE, ATE_2D, RPE, NEES
  │     │   └── 累积统计
  │     │
  │     └── 汇总: 该算法在该数据集上的所有运行统计
  │
  └── 3. 生成比较表格 (LaTeX格式):
        ATE/NESS 表格 + RPE 表格
```

### 7.3 RMSE 时序分析

```cpp
// 仅使用在所有运行中都有误差的时间点
for (auto &elm : rmse_dataset) {
    if (elm.second.first.values.size() == num_runs) {
        // 该时间点在所有运行中都有数据
        elm.second.first.calculate();  // 计算该时间点的RMSE
        rmse_ori.timestamps.push_back(elm.first);
        rmse_ori.values.push_back(elm.second.first.rmse);
    }
}
```

---

## 8. 其他评估工具

### 8.1 error_simulation

专门用于仿真模式评估：
- 可以评估完整状态（不仅位姿，还有速度/偏置/标定）
- 使用 `ResultSimulation` 类代替 `ResultTrajectory`
- 支持全状态 NEES 评估

### 8.2 pose_to_file

订阅 ROS 位姿话题，记录为评估用的文本文件：
```cpp
// 订阅 /ov_msckf/poseimu (PoseWithCovarianceStamped)
// 或 /ov_msckf/posegt (PoseStamped)
// 输出格式: timestamp px py pz qx qy qz qw cov...
```

### 8.3 时间性能分析

| 工具 | 功能 |
|------|------|
| `timing_flamegraph` | 生成火焰图输入，分析各函数耗时占比 |
| `timing_histogram` | 生成直方图数据，分析处理时间分布 |
| `timing_percentages` | 计算跟踪/传播/MSCKF/SLAM/边缘化各阶段的耗时百分比 |

时间统计文件格式 (由 VioManager 输出):
```
# timestamp (sec),tracking,propagation,msckf update,slam update,slam delayed,re-tri & marg,total
timestamp, time_track, time_prop, time_msckf, [time_slam_update, time_slam_delay,] time_marg, time_total
```

---

## 9. 完整评估流程总结

```
┌──────────────────────────────────────────────────────────┐
│                   评估工作流                              │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  1. 运行 VIO (任一种模式)                                 │
│     ├─ subscribe: roslaunch + rosbag play                │
│     ├─ serial: roslaunch (自动读取bag)                   │
│     └─ simulation: roslaunch (自动仿真)                  │
│                                                          │
│  2. 获取轨迹数据                                          │
│     ├─ dosave=true → pose_to_file 自动记录               │
│     └─ 或 rosbag record /ov_msckf/poseimu               │
│                                                          │
│  3. 运行评估                                              │
│     ├─ 单次: error_singlerun posyaw gt.txt est.txt       │
│     ├─ 批量: error_dataset posyaw gt.txt algorithms/     │
│     └─ 仿真: error_simulation                            │
│                                                          │
│  4. 解读结果                                              │
│     ├─ ATE: 全局一致性 (整体漂移)                         │
│     ├─ RPE: 局部精度 (短期漂移)                           │
│     └─ NEES: 滤波器一致性 (协方差质量)                     │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 10. 参考

- Zhang & Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018
- Umeyama, "Least-squares estimation of transformation parameters between two point patterns", TPAMI 1991
- rpg_trajectory_evaluation: https://github.com/uzh-rpg/rpg_trajectory_evaluation
