# run_simulation.cpp vs run_subscribe_msckf.cpp 详细对比分析

## 概述

两个文件都是 OpenVINS 的入口点，但服务不同场景：
- **run_simulation.cpp**: 仿真环境，使用 `Simulator` 生成完全综合的传感器数据
- **run_subscribe_msckf.cpp**: 实时环境，通过 ROS topic 订阅真实传感器数据

---

## 1. 系统初始化阶段

### 1.1 run_simulation.cpp 初始化（第 56-132 行）

```cpp
// ① 创建核心模块
sim = std::make_shared<Simulator>(params);           // 仿真器
sys = std::make_shared<VioManager>(params);         // VIO 管理器
viz = std::make_shared<ROS1Visualizer>(nh, sys, sim);  // 可视化（传入 sim 对象）

// ② 获取初始状态（ground truth）
double next_imu_time = sim->current_timestamp() + 1.0 / params.sim_freq_imu;
Eigen::Matrix<double, 17, 1> imustate;
bool success = sim->get_state(next_imu_time, imustate);

// ③ 用地面真值初始化滤波器
imustate(0, 0) -= sim->get_true_parameters().calib_camimu_dt;  // 转换到 IMU 时间
sys->initialize_with_gt(imustate);
```

**关键特性**：
- 调用 `Simulator::get_state()` 获取真值 17×1 状态向量：
  ```
  [t, q_x, q_y, q_z, q_w, p_x, p_y, p_z, v_x, v_y, v_z, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z]
  ```
- 使用 `initialize_with_gt()` **直接跳过** 初始化器（InertialInitializer），直接设置状态和协方差
- 相机 IMU 时间偏移 `calib_camimu_dt` 已从 Simulator 中提取

### 1.2 run_subscribe_msckf.cpp 初始化（第 43-94 行）

```cpp
// ① 创建核心模块
sys = std::make_shared<VioManager>(params);         // VIO 管理器
viz = std::make_shared<ROS1Visualizer>(nh, sys);   // 可视化（不传 sim）
viz->setup_subscribers(parser);                     // 设置 ROS 订阅

// ② 启动异步监听（ROS spinner）
ros::AsyncSpinner spinner(0);
spinner.start();
ros::waitForShutdown();
```

**关键特性**：
- 创建时 **不** 传递 Simulator 对象到 Visualizer
- 不进行任何初始化，等待订阅回调触发
- 通过 `setup_subscribers()` 创建以下订阅：
  - IMU topic: `callback_inertial()`
  - 相机 topic: `callback_monocular()` 或 `callback_stereo()`

---

## 2. 数据处理管道

### 2.1 run_simulation.cpp 同步主循环（第 134-182 行）

```cpp
while (sim->ok() && ros::ok()) {
  // ═════════════════════════════════════════════════════════════════
  // 阶段 1：获取下一个 IMU 测量
  // ═════════════════════════════════════════════════════════════════
  ov_core::ImuData message_imu;
  bool hasimu = sim->get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
  
  if (hasimu) {
    // ① 直接调用 feed_measurement_imu()
    sys->feed_measurement_imu(message_imu);
    
    // ② 立即可视化（高频显示）
    viz->visualize_odometry(message_imu.timestamp);
  }

  // ═════════════════════════════════════════════════════════════════
  // 阶段 2：获取下一个相机帧
  // ═════════════════════════════════════════════════════════════════
  double time_cam;
  std::vector<int> camids;
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
  bool hascam = sim->get_next_cam(time_cam, camids, feats);
  
  if (hascam) {
    // ① 处理前一帧（存在缓冲时）
    if (buffer_timecam != -1) {
      sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
      viz->visualize();  // 可视化
    }
    
    // ② 缓存当前帧用于下一次迭代
    buffer_timecam = time_cam;
    buffer_camids = camids;
    buffer_feats = feats;
  }
}
```

**工作流**：

```
Simulator
    │
    ├─→ get_next_imu()  ──→ message_imu
    │    (内部：BsplineSE3::get_acceleration + 传感器噪声)
    │
    └─→ get_next_cam()  ──→ message_cam
         (内部：BsplineSE3::get_pose + 点云投影)

feed_measurement_imu(message_imu)
    │
    └─→ VioManager::feed_measurement_imu()
        └─→ Propagator::propagate_and_clone()  [IMU 积分与克隆增强]

feed_measurement_simulation(time_cam, camids, feats)
    │
    └─→ VioManager::feed_measurement_simulation()
        └─→ TrackSIM::feed_measurement_simulation()  [模拟特征输入]
        └─→ track_image_and_update()  [特征更新]
```

**特点**：
- **同步顺序处理**：IMU 和相机数据按时间顺序逐个处理
- **前后缓存相机帧**：延后一帧处理，保证时间顺序正确
- **无时间同步复杂性**：由 Simulator 保证时序

### 2.2 run_subscribe_msckf.cpp 异步回调管道

#### 2.2.1 IMU 回调处理（ROS1Visualizer::callback_inertial）

```cpp
void ROS1Visualizer::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {
  // ═════════════════════════════════════════════════════════════════
  // 阶段 1：转换 ROS 消息格式
  // ═════════════════════════════════════════════════════════════════
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, 
               msg->angular_velocity.y, 
               msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x,
               msg->linear_acceleration.y,
               msg->linear_acceleration.z;

  // ═════════════════════════════════════════════════════════════════
  // 阶段 2：立即推送 IMU 数据到 VioManager
  // ═════════════════════════════════════════════════════════════════
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);

  // ═════════════════════════════════════════════════════════════════
  // 阶段 3：异步处理相机队列（确保相机在 IMU 之后）
  // ═════════════════════════════════════════════════════════════════
  if (thread_update_running)
    return;  // 线程已运行，直接返回
  
  thread_update_running = true;
  std::thread thread([&] {
    std::lock_guard<std::mutex> lck(camera_queue_mtx);
    
    // 计算 IMU 时间对应的相机时间
    double timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
    
    // 条件：处理所有时间早于 IMU 时间的相机帧
    while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
      auto rT0_1 = boost::posix_time::microsec_clock::local_time();
      
      // 调用相机更新
      _app->feed_measurement_camera(camera_queue.at(0));
      visualize();
      
      camera_queue.pop_front();
      
      auto rT0_2 = boost::posix_time::microsec_clock::local_time();
      double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
      PRINT_INFO("[TIME]: %.4f seconds (%.1f hz, %.2f ms behind)\n", 
                 time_total, 1.0 / time_total, 
                 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp));
    }
    thread_update_running = false;
  });

  // 单线程或多线程
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();  // 阻塞等待
  } else {
    thread.detach();  // 异步运行
  }
}
```

#### 2.2.2 相机回调处理（ROS1Visualizer::callback_monocular）

```cpp
void ROS1Visualizer::callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0) {
  // ═════════════════════════════════════════════════════════════════
  // 阶段 1：频率过滤（跳过太高频的帧）
  // ═════════════════════════════════════════════════════════════════
  double timestamp = msg0->header.stamp.toSec();
  double time_delta = 1.0 / _app->get_params().track_frequency;
  
  if (camera_last_timestamp.find(cam_id0) != camera_last_timestamp.end() && 
      timestamp < camera_last_timestamp.at(cam_id0) + time_delta) {
    return;  // 帧太密集，丢弃
  }
  camera_last_timestamp[cam_id0] = timestamp;

  // ═════════════════════════════════════════════════════════════════
  // 阶段 2：转换图像格式（ROS → OpenCV）
  // ═════════════════════════════════════════════════════════════════
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // ═════════════════════════════════════════════════════════════════
  // 阶段 3：创建相机数据包
  // ═════════════════════════════════════════════════════════════════
  ov_core::CameraData message;
  message.timestamp = cv_ptr->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.images.push_back(cv_ptr->image.clone());

  // 如果使用掩码，加载掩码；否则为空
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
  } else {
    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }

  // ═════════════════════════════════════════════════════════════════
  // 阶段 4：加入优先级队列（按时间戳排序）
  // ═════════════════════════════════════════════════════════════════
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());  // 按时间排序
}
```

**工作流**：

```
ROS IMU Topic                          ROS Camera Topic
       │                                       │
       └──→ callback_inertial()                └──→ callback_monocular()
           │                                       │
           ├─→ feed_measurement_imu()              ├─→ 加入队列
           │   └─→ VioManager::feed_measurement_imu()  └─→ sorted camera_queue
           │       └─→ Propagator::propagate_and_clone()
           │
           └──→ 异步线程
               │
               ├─→ 等待相机时间 < IMU 时间
               │
               └─→ while camera_queue.at(0).timestamp < timestamp_imu_inC:
                   └─→ feed_measurement_camera()
                       └─→ VioManager::feed_measurement_camera()
                           └─→ TrackBase::track()  [真实特征跟踪]
                           └─→ track_image_and_update()
```

**特点**：
- **异步回调驱动**：IMU 和相机各自独立回调
- **动态时序同步**：通过 IMU 时间戳触发相机处理
- **优先级队列管理**：相机帧缓存并自动排序
- **多线程并发**：可选单线程或多线程处理

---

## 3. 核心差异总结

| 维度 | run_simulation.cpp | run_subscribe_msckf.cpp |
|-----|-------------------|------------------------|
| **数据来源** | Simulator 生成 | ROS topic 订阅 |
| **时间控制** | 同步主循环 | 异步回调 |
| **初始化方式** | `initialize_with_gt()`（直接跳过初始化器） | `try_to_initialize()`（数据驱动初始化） |
| **IMU 处理** | 顺序调用 `get_next_imu()` | 异步 `callback_inertial()` |
| **相机处理** | 单帧缓冲，同步 | 优先级队列，异步 |
| **特征跟踪** | `TrackSIM`（模拟跟踪，直接输入） | `TrackBase`（真实跟踪，图像处理） |
| **时间同步** | Simulator 保证 | 手动检查 `timestamp_imu_inC` |
| **地面真值** | 同时运行，用于评估 | 可选离线加载 |
| **参数来源** | 配置文件 + Simulator | 配置文件 + ROS 参数服务器 |

---

## 4. 数据处理详细流程对比

### 4.1 IMU 数据处理

#### 仿真情况（run_simulation.cpp）

```
Simulator::get_next_imu()
  ↓
1. 计算 IMU 时间：timestamp_last_imu += 1.0 / sim_freq_imu
2. 调用 BsplineSE3::get_acceleration()
   ├─ 获取四个控制点位姿
   ├─ 计算三次 B 样条插值
   └─ 返回 (R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG)
3. 变换到 IMU 坐标系
   ├─ accel_inI = R_GtoI * (a_IinG + gravity)
   └─ omega_inI = w_IinI
4. 应用 IMU 内参失真
   ├─ omega_inGYRO = T_w * q_GYROtoIMU.inv() * omega_inI + T_g * accel_inI
   └─ accel_inACC = T_a * q_ACCtoIMU.inv() * accel_inI
5. 加入偏置和噪声
   ├─ wm = omega_inGYRO + bias_gyro + N(0, sigma_w)
   └─ am = accel_inACC + bias_accel + N(0, sigma_a)
  ↓
feed_measurement_imu(message_imu)
  ↓
VioManager::feed_measurement_imu()
  ├─ 调用 Propagator::select_imu_readings()
  ├─ 调用 Propagator::propagate_and_clone()
  │  └─ 在最后一个克隆时刻和当前 IMU 时间之间执行离散 IMU 积分
  └─ 更新状态协方差
```

**公式**：

从 B 样条得到的真实加速度：
$$a_{true}(t) = R_{GtoI}(t) \cdot (a_{world}(t) + \mathbf{g})$$

其中重力 $\mathbf{g} = [0, 0, 9.81]^T$

IMU 测量模型：
$$\mathbf{w}_m = \mathbf{T}_w \cdot R_{GYROtoIMU}^T \cdot \boldsymbol{\omega}_{true} + \mathbf{T}_g \cdot \mathbf{a}_{true} + \mathbf{b}_g + \mathbf{n}_w$$
$$\mathbf{a}_m = \mathbf{T}_a \cdot R_{ACCtoIMU}^T \cdot \mathbf{a}_{true} + \mathbf{b}_a + \mathbf{n}_a$$

其中 $\mathbf{T}_g$、$\mathbf{T}_w$、$\mathbf{T}_a$ 是 IMU 内参矩阵。

#### 实时情况（run_subscribe_msckf.cpp）

```
ROS IMU Topic
  ↓
callback_inertial(sensor_msgs::Imu::ConstPtr msg)
  ↓
1. 转换格式
   ├─ message.timestamp = msg->header.stamp.toSec()
   ├─ message.wm = [msg->angular_velocity.{x,y,z}]
   └─ message.am = [msg->linear_acceleration.{x,y,z}]
2. 立即推送
   └─ feed_measurement_imu(message)
  ↓
VioManager::feed_measurement_imu()
  ├─ 同上（仿真路径）
  └─ 然后检查是否有缓存相机帧需要处理
```

**区别**：
- 实时情况：直接使用硬件传感器的原始输出（已包含噪声、偏置、失真）
- 仿真情况：从真值轨迹综合生成测量值

---

### 4.2 相机数据处理

#### 仿真情况（run_simulation.cpp）

```
Simulator::get_next_cam()
  ↓
1. 计算相机时间：timestamp_last_cam += 1.0 / sim_freq_cam
2. 调用 BsplineSE3::get_pose()
   └─ 返回 (R_GtoI, p_IinG)
3. 遍历点云特征
   ├─ 变换到相机坐标系
   │  └─ p_FinC = R_ItoC * R_GtoI * (feat - p_IinG) + p_IinC
   ├─ 检查深度和视场内
   ├─ 投影到像素坐标
   │  └─ uv_norm = [p_FinC.x/p_FinC.z, p_FinC.y/p_FinC.z]
   ├─ 应用相机失真
   │  └─ uv_dist = camera->distort_f(uv_norm)
   └─ 收集投影特征 {feat_id, uv_dist}
4. 加入测量噪声
   └─ uv_noisy = uv_dist + N(0, sigma_pix)
  ↓
缓冲相机帧（延后处理）
  ↓
feed_measurement_simulation(time_cam, camids, feats)
  ↓
VioManager::feed_measurement_simulation()
  ├─ 检查/创建 TrackSIM 对象
  ├─ 调用 TrackSIM::feed_measurement_simulation()
  │  └─ 直接存储特征（无跟踪）
  └─ 调用 track_image_and_update()
     ├─ 特征匹配
     ├─ 特征初始化
     └─ MSCKF/SLAM 更新
```

**公式**：

世界坐标 → 相机坐标：
$$\mathbf{p}_{FinC} = R_{ItoC} \cdot R_{GtoI}(t) \cdot (\mathbf{p}_{F} - \mathbf{p}_{IinG}(t)) + \mathbf{p}_{IinC}$$

投影到归一化像素：
$$\mathbf{u}_{norm} = \frac{1}{p_{FinC,z}} \begin{bmatrix} p_{FinC,x} \\ p_{FinC,y} \end{bmatrix}$$

应用失真（例如 Brown-Conrady 模型）：
$$\mathbf{u}_{dist} = f(\mathbf{u}_{norm}; k_1, k_2, k_3, k_4, ...)$$

带噪声的测量：
$$\mathbf{z}_{cam} = \mathbf{u}_{dist} + \mathbf{n}_{cam}, \quad \mathbf{n}_{cam} \sim \mathcal{N}(0, \sigma_{pix}^2 I)$$

#### 实时情况（run_subscribe_msckf.cpp）

```
ROS Camera Topic
  ↓
callback_monocular(sensor_msgs::Image msg, cam_id)
  ↓
1. 频率过滤
   └─ 检查 timestamp - last_timestamp > 1/track_frequency
2. 图像格式转换（ROS → OpenCV）
   └─ cv_bridge::toCvShare() → cv::Mat
3. 创建相机数据包
   ├─ CameraData.timestamp = msg->header.stamp.toSec()
   ├─ CameraData.sensor_ids = [cam_id]
   ├─ CameraData.images = [cv::Mat]
   └─ CameraData.masks = [mask] 或空
4. 加入优先级队列并排序
   └─ camera_queue.push_back(message)
   └─ std::sort(camera_queue.begin(), camera_queue.end())
  ↓
由 IMU 回调触发处理
  ↓
feed_measurement_camera(camera_queue.at(0))
  ↓
VioManager::feed_measurement_camera()
  ├─ 调用 TrackBase::track()
  │  ├─ 特征检测（FAST 等）
  │  ├─ 特征跟踪（Lucas-Kanade 等）
  │  └─ 特征描述符匹配
  └─ 调用 track_image_and_update()
     ├─ 特征初始化
     └─ MSCKF/SLAM 更新
```

**区别**：
- 实时情况：原始图像作为输入，需要完整的特征检测和跟踪算法
- 仿真情况：直接提供已投影的特征位置，跳过图像处理步骤

---

## 5. 时序同步机制

### 5.1 仿真中的时序（同步）

```
时间轴：
────────────────────────────────────────→

t₀          t₀ + Δt_imu    t₀ + 2Δt_imu   ...
│               │               │
IMU #1      IMU #2         IMU #3
│
├─ 相机帧（时间早于 IMU #2）
│  ├─ get_next_cam() 检查  
│  ├─ camera_time < imu_time?  ✓
│  └─ 处理

主循环每迭代：
1. 获取 IMU
2. 处理 IMU
3. 检查并获取相机
4. 如果有前一帧，处理它
5. 缓存当前帧用于下一迭代
```

**保证**：单调递增时间，由 Simulator 内部维护

### 5.2 实时中的时序（异步但同步）

```
事件线：

时间 t₁: IMU 消息到达
         ├─ callback_inertial()
         ├─ feed_measurement_imu()
         └─ 启动异步线程检查相机队列
            │
            └─ 如果 camera.timestamp < (imu_time - calib_dt_CAMtoIMU)
               └─ 处理相机帧

时间 t₂: 相机消息到达（可能 t₂ > t₁）
         └─ callback_monocular()
            ├─ 转换图像
            ├─ 加入 camera_queue
            ├─ 排序队列
            └─ 返回（等待 IMU 触发）

时间 t₃: 下一个 IMU 消息
         └─ callback_inertial()
            └─ 异步线程检查相机
               └─ 如果 camera.timestamp < (t₃ - calib_dt)
                  └─ 处理相机
```

**关键同步条件**：
```cpp
// IMU 时间转换到相机坐标系
double timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);

// 只处理时间早于当前 IMU 时间的相机帧
while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
  _app->feed_measurement_camera(camera_queue.at(0));
  camera_queue.pop_front();
}
```

其中：
- `_calib_dt_CAMtoIMU` = 相机 ISO 时间 - IMU 时间（通常 ≠ 0）

---

## 6. 初始化对比

### 6.1 仿真初始化（initialize_with_gt）

```cpp
// 直接设置 IMU 状态（17×1）
state->_imu->set_value(imustate.block(1, 0, 16, 1));
state->_imu->set_fej(imustate.block(1, 0, 16, 1));

// 设置初始协方差
Cov = [
  σ_q² × I₃         // 旋转方差
  σ_p² × I₃         // 位置方差
  σ_v² × I₃         // 速度方差
  σ_bg² × I₃        // 陀螺仪偏置方差
  σ_ba² × I₃        // 加速度计偏置方差
]

// 标记已初始化
is_initialized_vio = true;
state->_timestamp = imustate(0, 0);
```

**优点**：
- 跳过初始化阶段，立即开始 VIO
- 初始协方差精确已知
- 适合演示和基准测试

**缺点**：
- 不检验实际传感器数据的一致性
- 不进行真正的初始化 EKF

### 6.2 实时初始化（try_to_initialize）

```cpp
// 从 InertialInitializer 运行初始化
bool success = initializer->initialize(
  timestamp, covariance, order, state->_imu, wait_for_jerk
);

if (success) {
  // 设置状态（已在初始化器中设置）
  StateHelper::set_initial_covariance(state, covariance, order);
  
  is_initialized_vio = true;
  state->_timestamp = timestamp;
} else {
  // 等待更多数据，再次尝试
  return false;
}
```

**算法**（简化）：
1. 积累 IMU 测量数据
2. 计算平均加速度（重力估计）
3. 等待"剧烈运动"（jerk）以确定方向
4. 初始化 EKF 状态和协方差

**优点**：
- 完全数据驱动，不依赖外部信息
- 协方差从实际噪声估计得出

**缺点**：
- 初始化可能需要 0.5-5 秒
- 初期易失效

---

## 7. 参数获取来源

| 参数 | run_simulation | run_subscribe |
|-----|---------------|----------------|
| IMU 频率 | `params.sim_freq_imu` | 从 ROS 消息时戳推断 |
| 相机频率 | `params.sim_freq_cam` | 从 ROS 消息时戳推断 |
| 相机内参 | `params.camera_intrinsics` | 从配置文件 |
| IMU 内参 | `params.vec_dw`, `params.vec_da` | 从配置文件 |
| 相机-IMU 偏置 | `sim->get_true_parameters().calib_camimu_dt` | `_app->get_state()->_calib_dt_CAMtoIMU` |
| 噪声参数 | `params.imu_noises.sigma_w` 等 | 从初始化协方差 |
| 地面真值 | Simulator 内部维护 | 可选：离线加载文件 |

---

## 8. 特征跟踪对比

### 8.1 仿真中的 TrackSIM

```cpp
// 直接接收特征位置（无跟踪）
TrackSIM::feed_measurement_simulation(timestamp, camids, feats)
  ├─ 特征直接存储到数据库
  └─ 无特征检测、无描述符匹配
```

**优点**：
- 完全确定的特征对应（无跟踪错误）
- 计算高效

**缺点**：
- 不反映真实跟踪困难

### 8.2 实时中的 TrackBase

```cpp
// 从图像跟踪特征
TrackBase::track(CameraData& message)
  ├─ 特征检测（FAST + NEON 加速）
  ├─ 描述符计算（512-bit BRIEF）
  ├─ 特征匹配
  │  ├─ 相同相机内：Lucas-Kanade 光流
  │  └─ 不同相机间：描述符匹配
  └─ 异常值拒绝（RANSAC）
```

**优点**：
- 真实的跟踪鲁棒性测试

**缺点**：
- 计算开销大
- 可能出现跟踪失败

---

## 9. 工作流对比图

### 仰制流程图
```
Simulator
    │
    ├─→ feed_trajectory()
    │   └─→ BsplineSE3::control_points
    │
    ├─ while run:
    │   ├─ get_next_imu()
    │   │  └─ BsplineSE3::get_acceleration()
    │   │
    │   └─ get_next_cam()
    │      └─ BsplineSE3::get_pose()
    │
    ├─→ initialize_with_gt()
    │
    └─ main loop:
        ├─ feed_measurement_imu()
        │  └─ propagate_and_clone()
        │
        └─ feed_measurement_simulation()
           ├─ TrackSIM::feed_measurement_simulation()
           └─ track_image_and_update()
```

### 实时流程图
```
ROS Topic Remapping
    │
    ├─→ IMU Topic
    │   └─→ callback_inertial()
    │       ├─ feed_measurement_imu()
    │       │  └─ propagate_and_clone()
    │       │
    │       └─ async thread:
    │           └─ while camera.timestamp < imu_time - calib_dt:
    │              └─ feed_measurement_camera()
    │                 ├─ TrackBase::track()
    │                 └─ track_image_and_update()
    │
    └─→ Camera Topic
        └─→ callback_monocular()
            ├─ convert image (cv_bridge)
            └─ push to camera_queue
```

---

## 总结

| 任务 | run_simulation | run_subscribe |
|-----|---------------|----------------|
| **调试 / 基准** | ✓ 完美 | ✗ 实时性能波动 |
| **算法验证** | ✓ 确定性结果 | ✗ 受噪声影响 |
| **实际部署** | ✗ 无法部署 | ✓ 可直接部署 |
| **性能评估** | ✓ 有地面真值 | ✓ 可加载离线GT |
| **鲁棒性测试** | ✓ 可配置噪声 | ✓ 真实噪声 |
| **算法开发** | ✓ 快速迭代 | ✗ 初始化耗时 |
