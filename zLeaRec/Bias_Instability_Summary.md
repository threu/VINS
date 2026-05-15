# Simulator 中的零偏不稳定性噪声梳理

## 1. 零偏模型概述

Simulator 中使用的零偏模型是**随机游走（Random Walk）模型**，这是IMU传感器常用的建模方式。

### 1.1 零偏类型

系统中维护两种零偏：
- **陀螺仪零偏**（`true_bias_gyro`）：3×1向量，单位 rad/s
- **加速度计零偏**（`true_bias_accel`）：3×1向量，单位 m/s²

### 1.2 零偏参数

在 `NoiseManager` 中定义（位置：[src/open_vins/ov_msckf/src/utils/NoiseManager.h](src/open_vins/ov_msckf/src/utils/NoiseManager.h)）：

| 参数 | 含义 | 单位 | 默认值 |
|------|------|------|--------|
| `sigma_wb` | 陀螺仪随机游走噪声标准差 | rad/s²/√Hz | 1.9393e-05 |
| `sigma_ab` | 加速度计随机游走噪声标准差 | m/s³/√Hz | 3.0000e-03 |
| `sigma_w` | 陀螺仪白噪声标准差 | rad/s/√Hz | 1.6968e-04 |
| `sigma_a` | 加速度计白噪声标准差 | m/s²/√Hz | 2.0000e-03 |

---

## 2. 零偏不稳定性噪声的添加过程

### 2.1 核心实现位置

在 [Simulator.cpp](src/open_vins/ov_msckf/src/sim/Simulator.cpp) 的 `get_next_imu()` 函数中（第364-371行）：

```cpp
// Calculate the bias values for this IMU reading
// NOTE: we skip the first ever bias since we have already appended it
double dt = 1.0 / params.sim_freq_imu;
std::normal_distribution<double> w(0, 1);
if (has_skipped_first_bias) {

    // Move the biases forward in time (Random Walk Model)
    true_bias_gyro(0) += params.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(1) += params.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_gyro(2) += params.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
    
    true_bias_accel(0) += params.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(1) += params.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
    true_bias_accel(2) += params.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);

    // Append the current true bias to our history
    hist_true_bias_time.push_back(timestamp_last_imu);
    hist_true_bias_gyro.push_back(true_bias_gyro);
    hist_true_bias_accel.push_back(true_bias_accel);
}
has_skipped_first_bias = true;
```

### 2.2 随机游走模型公式

每个时间步的零偏更新遵循以下随机游走模型：

$$b_k = b_{k-1} + \sigma \sqrt{\Delta t} \cdot N(0,1)$$

其中：
- $b_k$：当前时刻的零偏
- $b_{k-1}$：上一时刻的零偏
- $\sigma$：随机游走噪声标准差（sigma_wb 或 sigma_ab）
- $\Delta t$：采样间隔 $1/f_{imu}$
- $N(0,1)$：独立的标准正态随机变量（由 `std::normal_distribution<double> w(0, 1)` 生成）

### 2.3 具体工作流程

1. **初始化**：`true_bias_accel` 和 `true_bias_gyro` 初始化为零向量
2. **跳过第一个零偏**：`has_skipped_first_bias` 标志用于跳过第一次更新
3. **每个IMU采样点**：
   - 为三个轴分别生成独立的正态随机数
   - 按照随机游走模型更新零偏
   - 保存零偏值到历史记录中
4. **应用零偏到测量**（第380-385行）：
   ```cpp
   wm(0) = omega_inGYRO(0) + true_bias_gyro(0) + params.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
   wm(1) = omega_inGYRO(1) + true_bias_gyro(1) + params.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
   wm(2) = omega_inGYRO(2) + true_bias_gyro(2) + params.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
   am(0) = accel_inACC(0) + true_bias_accel(0) + params.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
   am(1) = accel_inACC(1) + true_bias_accel(1) + params.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
   am(2) = accel_inACC(2) + true_bias_accel(2) + params.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
   ```

---

## 3. 数据结构

### 3.1 零偏历史记录

系统维护零偏的历史记录用于状态查询（在 [Simulator.h](src/open_vins/ov_msckf/src/sim/Simulator.h)）：

```cpp
/// Our running acceleration bias
Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

/// Our running gyroscope bias
Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

// Our history of true biases
bool has_skipped_first_bias = false;
std::vector<double> hist_true_bias_time;        // 时间戳记录
std::vector<Eigen::Vector3d> hist_true_bias_accel;  // 加速度计零偏历史
std::vector<Eigen::Vector3d> hist_true_bias_gyro;   // 陀螺仪零偏历史
```

### 3.2 零偏查询函数

`get_state()` 函数支持在任意时刻查询真实零偏，使用线性插值：

```cpp
// Interpolate our biases (they will be at every IMU timestep)
double lambda = (desired_time - hist_true_bias_time.at(id_loc)) / 
                (hist_true_bias_time.at(id_loc + 1) - hist_true_bias_time.at(id_loc));
Eigen::Vector3d true_bg_interp = (1 - lambda) * hist_true_bias_gyro.at(id_loc) + 
                                  lambda * hist_true_bias_gyro.at(id_loc + 1);
Eigen::Vector3d true_ba_interp = (1 - lambda) * hist_true_bias_accel.at(id_loc) + 
                                  lambda * hist_true_bias_accel.at(id_loc + 1);
```

---

## 4. 噪声模型总结

### 4.1 完整的IMU测量模型

最终的IMU测量值可以表示为：

$$\mathbf{w}_m = \mathbf{w}_{true} + \mathbf{b}_g + \mathbf{n}_w$$
$$\mathbf{a}_m = \mathbf{a}_{true} + \mathbf{b}_a + \mathbf{n}_a$$

其中：
- $\mathbf{w}_m$, $\mathbf{a}_m$：测量的角速度和线性加速度
- $\mathbf{w}_{true}$, $\mathbf{a}_{true}$：真实的角速度和线性加速度
- $\mathbf{b}_g$, $\mathbf{b}_a$：零偏（通过随机游走过程生成）
- $\mathbf{n}_w$, $\mathbf{n}_a$：白噪声

### 4.2 随机游走过程的性质

- **平稳性**：零偏不是静态常数，而是持续变化的
- **相关性**：相邻时刻的零偏高度相关（由 $\sqrt{\Delta t}$ 项决定）
- **易识别性**：长时间积分后，零偏会明显漂移，便于算法估计

### 4.3 配置参数来源

这些参数通常来自：
1. **Kalibr标定结果**：从IMU标定得到的参数
2. **IMU数据表**：制造商提供的规格
3. **实验测量**：通过Allan方差等方法测定

---

## 5. 关键代码流程图

```
get_next_imu() 被调用
    ↓
读取真实的角速度和加速度
    ↓
应用IMU内参变换（标定矩阵）
    ↓
更新零偏（随机游走模型） ← 零偏不稳定性噪声添加位置
    ├─ b_gyro(i) += sigma_wb * sqrt(dt) * N(0,1)
    └─ b_accel(i) += sigma_ab * sqrt(dt) * N(0,1)
    ↓
保存零偏到历史记录
    ↓
添加白噪声到测量值
    ├─ wm += sigma_w / sqrt(dt) * N(0,1)
    └─ am += sigma_a / sqrt(dt) * N(0,1)
    ↓
返回带噪声的IMU测量值
```

---

## 6. 相关参数配置

在 VioManager 初始化时，从配置文件加载（[VioManagerOptions.h](src/open_vins/ov_msckf/src/core/VioManagerOptions.h)，第159-162行）：

```yaml
# 配置文件格式（来自 relative_config_imu）
imu0:
  gyroscope_noise_density: 1.6968e-04          # sigma_w
  gyroscope_random_walk: 1.9393e-05            # sigma_wb
  accelerometer_noise_density: 2.0000e-3       # sigma_a
  accelerometer_random_walk: 3.0000e-03        # sigma_ab
```

**数据流**
真实加速度/角速度 → 应用标定 → 更新零偏（随机游走） → 加零偏 → 加白噪声 → 返回测量值

---

## 总结

1. **模型**：使用标准的**随机游走（Random Walk）过程**来模型化零偏不稳定性
2. **更新**：每个IMU采样点，零偏增量为 $\sigma \sqrt{\Delta t} \times N(0,1)$
3. **参数**：由配置文件指定，分别为 `sigma_wb`（陀螺）和 `sigma_ab`（加速度计）
4. **效果**：使零偏缓慢漂移，更逼真地模拟真实IMU传感器行为
5. **用途**：便于测试和验证VIO算法对零偏变化的鲁棒性
