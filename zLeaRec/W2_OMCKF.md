```cpp
#if ROS_AVAILABLE == 1
  viz = std::make_shared<ROS1Visualizer>(nh, sys);
  viz->setup_subscribers(parser);
#elif ROS_AVAILABLE == 2
  viz = std::make_shared<ROS2Visualizer>(node, sys);
  viz->setup_subscribers(parser);
#endif
```

```cpp
void ROS1Visualizer::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {

  // We need a valid parser
  assert(parser != nullptr);

  // Create imu subscriber (handle legacy ros param info)
  std::string topic_imu;
  _nh->param<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);
  PRINT_INFO("subscribing to IMU: %s\n", topic_imu.c_str());

  // Logic for sync stereo subscriber
  // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
  if (_app->get_params().state_options.num_cameras == 2) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    _nh->param<std::string>("topic_camera" + std::to_string(0), cam_topic0, "/cam" + std::to_string(0) + "/image_raw");
    _nh->param<std::string>("topic_camera" + std::to_string(1), cam_topic1, "/cam" + std::to_string(1) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(0), "rostopic", cam_topic0);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(1), "rostopic", cam_topic1);
    // Create sync filter (they have unique pointers internally, so we have to use move logic here...)
    auto image_sub0 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic0, 1);
    auto image_sub1 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic1, 1);
    auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
    sync->registerCallback(boost::bind(&ROS1Visualizer::callback_stereo, this, _1, _2, 0, 1));
    // Append to our vector of subscribers
    sync_cam.push_back(sync);
    sync_subs_cam.push_back(image_sub0);
    sync_subs_cam.push_back(image_sub1);
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic0.c_str());
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic1.c_str());
  } else {
    // Now we should add any non-stereo callbacks here
    for (int i = 0; i < _app->get_params().state_options.num_cameras; i++) {
      // read in the topic
      std::string cam_topic;
      _nh->param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
      parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
      // create subscriber
      subs_cam.push_back(_nh->subscribe<sensor_msgs::Image>(cam_topic, 10, boost::bind(&ROS1Visualizer::callback_monocular, this, _1, i)));
      PRINT_INFO("subscribing to cam (mono): %s\n", cam_topic.c_str());
    }
  }
}
```

```cpp
 sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);

 sync->registerCallback(boost::bind(&ROS1Visualizer::callback_stereo, this, _1, _2, 0, 1));

 subs_cam.push_back(_nh->subscribe<sensor_msgs::Image>(cam_topic, 10, boost::bind(&ROS1Visualizer::callback_monocular, this, _1, i)));
```
```cpp
void ROS1Visualizer::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);

  // If the processing queue is currently active / running just return so we can keep getting measurements
  // Otherwise create a second thread to do our update in an async manor
  // The visualization of the state, images, and features will be synchronous with the update!
  if (thread_update_running)
    return;
  thread_update_running = true;
  std::thread thread([&] {
    // Lock on the queue (prevents new images from appending)
    std::lock_guard<std::mutex> lck(camera_queue_mtx);

    // Count how many unique image streams
    std::map<int, bool> unique_cam_ids;
    for (const auto &cam_msg : camera_queue) {
      unique_cam_ids[cam_msg.sensor_ids.at(0)] = true;
    }

    // If we do not have enough unique cameras then we need to wait
    // We should wait till we have one of each camera to ensure we propagate in the correct order
    auto params = _app->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() == num_unique_cameras) {

      // Loop through our queue and see if we are able to process any of our camera measurements
      // We are able to process if we have at least one IMU measurement greater than the camera time
      double timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
        auto rT0_1 = boost::posix_time::microsec_clock::local_time();
        double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);
        _app->feed_measurement_camera(camera_queue.at(0));
        visualize();
        camera_queue.pop_front();
        auto rT0_2 = boost::posix_time::microsec_clock::local_time();
        double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
        PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);
      }
    }
    thread_update_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
}
```

```cpp
void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  propagator->feed_imu(message, oldest_time);

  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
  }

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}
```

### `viz->setup_subscribers(parser)` 的详细内容

`setup_subscribers` 函数负责设置 ROS 订阅器，订阅 IMU 和相机话题，并绑定相应的回调函数。以下是基于 ROS1Visualizer.cpp（ROS2 类似）的详细列出：

#### 1. **IMU 订阅设置**
   - **话题获取**：
     - 从 ROS 参数读取 `topic_imu`，默认值为 `"/imu0"`。
     - 使用 `parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);` 从配置文件解析外部参数。
   - **订阅器创建**：
     - `sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);`
     - 队列大小：1000。
     - 回调函数：`callback_inertial`。
   - **输出**：打印订阅信息 `"subscribing to IMU: %s\n"`。

#### 2. **相机订阅设置**
   - **判断相机数量**：
     - 如果 `num_cameras == 2`（立体模式）：
       - 读取两个相机话题：`topic_camera0` 和 `topic_camera1`，默认 `"/cam0/image_raw"` 和 `"/cam1/image_raw"`。
       - 使用 `parser->parse_external` 解析配置。
       - 创建同步订阅器：
         - `message_filters::Subscriber<sensor_msgs::Image>` 用于两个图像流。
         - `message_filters::Synchronizer<sync_pol>` 使用 `ApproximateTime` 策略（容差 10），回调 `callback_stereo`。
       - 存储到 `sync_cam` 和 `sync_subs_cam` 向量中。
       - 输出：打印立体订阅信息。
     - 否则（单目模式）：
       - 对每个相机（循环 `i` 从 0 到 `num_cameras-1`）：
         - 读取话题 `topic_camera{i}`，默认 `"/cam{i}/image_raw"`。
         - 创建独立订阅器：`subs_cam.push_back(_nh->subscribe<sensor_msgs::Image>(cam_topic, 10, boost::bind(&ROS1Visualizer::callback_monocular, this, _1, i)));`
         - 队列大小：10。
         - 回调函数：`callback_monocular`，传入相机 ID `i`。
         - 输出：打印单目订阅信息。

#### 回调函数与 `feed_measurement_imu` 的调用
- **IMU 回调**：`callback_inertial(const sensor_msgs::Imu::ConstPtr &msg)`
  - 将 ROS 消息转换为 `ov_core::ImuData`：
    - `message.timestamp = msg->header.stamp.toSec();`
    - `message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;`（角速度向量）
    - `message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;`（加速度向量）
  - 调用 `_app->feed_measurement_imu(message);`（间接调用 `VioManager::feed_measurement_imu`）。
  - 后续调用 `visualize_odometry(message.timestamp);`。
  - 如果更新线程未运行，则启动异步线程处理相机队列。
- **相机回调**：
  - `callback_stereo`：处理同步立体图像，调用 `callback_monocular` 或直接处理。
  - `callback_monocular`：处理单目图像，缓冲到队列，触发更新。

### 整个流程中涉及的公式计算

整个传感器数据处理流程涉及多个数学公式，主要在 IMU 传播、特征跟踪和 MSCKF 更新中。以下按阶段列出关键公式（基于工程代码和 VIO 理论）：

#### 1. **IMU 数据处理（传播阶段）**
   - **旋转更新**（四元数积分）：
     $$
     \dot{q} = \frac{1}{2} q \otimes \omega_{measured}
     $$
     其中 $ q $为四元数，$\omega_{measured} $为测量角速度。
   - **速度更新**：
     $$
     \dot{v} = a_{measured} + g - b_{accel}
     $$
     其中 $a_{measured} $为测量加速度，$ g $为重力向量，$b_{accel} $为加速度偏置。
   - **位置更新**：
     $$
     \dot{p} = v
     $$
     其中 $v $速度。
   - **协方差传播**（离散时间）：
     使用状态转移矩阵和过程噪声矩阵更新协方差$P $，涉及 IMU 噪声模型（陀螺仪和加速度计噪声）。

#### 2. **相机数据处理（特征跟踪）**
   - **KLT 光流跟踪**：
     最小化残差：
     $$
     \min \sum (I(x + u, y + v) - I(x, y))^2
     $$
     其中 $I $为图像强度，$u, v $为位移。
   - **描述符匹配**（ORB/BRISK）：
     使用汉明距离或相似度度量匹配特征描述符。

#### 3. **MSCKF 更新**
   - **特征三角化**：
     使用多视角几何从多个相机姿态计算 3D 点：
     $$
     \mathbf{p}^w = triangulate(\mathbf{u}_1, \mathbf{u}_2, \dots, \mathbf{u}_n, \mathbf{T}_{c1}^w, \mathbf{T}_{c2}^w, \dots)
     $$
     其中 $\mathbf{u}_i $为图像坐标，$\mathbf{T}_{ci}^w$为相机到世界的变换。
   - **测量残差**：
     $$
     \mathbf{r} = \mathbf{z} - \hat{\mathbf{z}}
     $$
     其中$\mathbf{z} $为测量值，$\hat{\mathbf{z}} $为预测值。
   - **雅可比矩阵**：
     残差对状态的导数，用于 EKF 更新：
     $$
     H = \frac{\partial \mathbf{r}}{\partial \mathbf{x}}
     $$
   - **空空间投影**：
     投影雅可比以消除特征参数：
     $$
     H_{proj} = H (I - V V^T)
     $$
     其中 $V $为特征参数的基向量。
   - **卡尔曼增益和更新**：
     $$
     K = P H^T (H P H^T + R)^{-1}
     $$
     $$
     \mathbf{x} = \mathbf{x} + K \mathbf{r}
     $$
     $$
     P = (I - K H) P
     $$
     其中 $R$为测量噪声协方差。

#### 4. **SLAM 更新**
   - **延迟初始化**：新特征在积累足够观测后初始化，使用类似三角化的公式。
   - **锚定管理**：特征锚定到相机姿态，更新时考虑锚点变换。



---

## 从 run_subscribe_msckf.cpp 到 IMU 处理的完整调用链

这个文件本身不直接计算 IMU 数据，而是负责：
1. 初始化 ROS 节点；
2. 创建 `VioManager`；
3. 创建可视化器 `ROS1Visualizer` / `ROS2Visualizer`；
4. 调用 `viz->setup_subscribers(parser)` 建立 IMU / 相机订阅。

所以 IMU 处理链路是：

run_subscribe_msckf.cpp
→ `viz->setup_subscribers(parser)`
→ `ROS1Visualizer::callback_inertial(...)` / `ROS2Visualizer::callback_inertial(...)`
→ `VioManager::feed_measurement_imu(...)`
→ `Propagator::feed_imu(...)` （且在初始化阶段还会进入 `InertialInitializer::feed_imu(...)`）

---

## 1. run_subscribe_msckf.cpp 的角色

在 `main()` 中，这几句是关键：

```cpp
sys = std::make_shared<VioManager>(params);
viz = std::make_shared<ROS1Visualizer>(nh, sys);
viz->setup_subscribers(parser);
```

- `sys`：核心滤波器和状态管理器；
- `viz`：负责 ROS 订阅、回调、可视化；
- `setup_subscribers(parser)`：把 IMU 话题绑定到 `callback_inertial`。

---

## 2. `setup_subscribers(parser)` 具体做了什么

以 ROS1 为例，ROS1Visualizer.cpp：

```cpp
std::string topic_imu;
_nh->param<std::string>("topic_imu", topic_imu, "/imu0");
parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);
```

这段意思是：
- 从 ROS 参数读取 `topic_imu`；
- 默认 `"/imu0"`；
- 订阅该 IMU 话题；
- 回调函数是 `callback_inertial`。

---

## 3. `callback_inertial()`：IMU ROS 数据如何进入 VIO

`ROS1Visualizer::callback_inertial(...)` 先把 ROS IMU 消息转成内部格式：

```cpp
ov_core::ImuData message;
message.timestamp = msg->header.stamp.toSec();
message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
```

然后直接调用：

```cpp
_app->feed_measurement_imu(message);
```

这是 IMU 数据进入 VIO 的真实起点。

---

## 4. `VioManager::feed_measurement_imu(...)` 的处理逻辑

在 VioManager.cpp，这个函数负责把 IMU 数据分发到三个模块：

```cpp
propagator->feed_imu(message, oldest_time);

if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
}

if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
}
```

### 4.1 `oldest_time` 的选取
- 如果已经初始化，则：
  - `oldest_time = state->margtimestep();`
- 如果尚未初始化，则：
  - `oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;`

也就是说：
- 初始化前保留最近一段时间的 IMU；
- 初始化后保留到最早需要边缘化的 clone 时间。

---

## 5. `Propagator::feed_imu(...)`：IMU 数据缓存与清理

这是最底层的 IMU 数据接收代码，位于 `state/Propagator.h`：

```cpp
void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    imu_data.emplace_back(message);
    clean_old_imu_measurements(oldest_time - 0.10);
}
```

它做两件事：
1. 把 IMU 测量加入 `imu_data` 缓冲；
2. 删除早于 `oldest_time - 0.10` 的旧数据。

---

## 6. `InertialInitializer::feed_imu(...)`：初始化阶段也保存 IMU

如果系统还未初始化，IMU 还会进初始化器：

```cpp
imu_data->emplace_back(message);
if (oldest_time != -1) {
    ... erase older than oldest_time ...
}
```

这保证了在 VIO 初始化阶段，IMU 数据也可用于：
- 重力估计；
- 速度、偏置、尺度初始化；
- feature / IMU 时间对齐。

---

## 7. 何时真正“用到”这些 IMU 数据？

单纯接收 IMU 后，真正状态传播是在相机数据到来时才触发：
- 在 `callback_inertial()` 内，IMU 每到一条会尝试启动相机处理线程；
- 该线程检查相机队列是否有可处理的图像数据；
- 若可处理，则调用 `_app->feed_measurement_camera(...)`；
- 相机更新内部会调用 `propagator->propagate_and_clone(state, message.timestamp)`。

因此，这个系统是典型的：
- IMU 异步缓存；
- 相机帧到来时按时间顺序用 IMU 做预测；

---

## 8. `propagate_and_clone(...)`：IMU 预测的核心

这是真正把 IMU 数据转成状态预测的代码，位于 `state/Propagator.cpp`。

### 8.1 选取 IMU 区间
它先计算：
- `time0 = state->_timestamp + last_prop_time_offset`
- `time1 = timestamp + t_off_new`

其中：
- `state->_timestamp` 是当前滤波器状态的最后时间；
- `t_off_new = state->_calib_dt_CAMtoIMU->value()(0)` 是相机-IMU 时间偏移。

然后从缓存中选择：
```cpp
prop_data = Propagator::select_imu_readings(imu_data, time0, time1);
```

这会包括边界插值，使 IMU 区间精确对齐到相机时间。

### 8.2 逐段离散预测
对每个相邻 IMU 读数区间 `prop_data[i]` 到 `prop_data[i+1]`：

```cpp
predict_and_compute(state, prop_data.at(i), prop_data.at(i + 1), F, Qdi);
```

- `F`：状态转移矩阵
- `Qdi`：该区间的离散过程噪声

然后累计：
```cpp
Phi_summed = F * Phi_summed;
Qd_summed = F * Qd_summed * F.transpose() + Qdi;
```

最终得到整个时间段的总状态转移 `Phi_summed` 和总噪声 `Qd_summed`。

---

## 9. 具体的 IMU 预测公式

Propagator.h 中明确给出离散传播公式：

- 姿态更新：
  $$
  {}^{I_{k+1}}_{G}\hat{\bar{q}}
  = \exp\left(\frac{1}{2}\Omega(\omega_{m,k}-\hat b_g)\Delta t\right) {}^{I_{k}}_{G}\hat{\bar{q}}
  $$

- 速度更新：
  $$
  ^G\hat{v}_{k+1} = {}^G\hat{v}_{k}
  - {}^G g \Delta t
  + {}^{I_k}_G \hat{R}^\top (a_{m,k} - \hat b_a) \Delta t
  $$

- 位置更新：
  $$
  ^G\hat{p}_{k+1} = {}^G\hat{p}_k
  + {}^G\hat{v}_k \Delta t
  - \frac{1}{2} {}^G g \Delta t^2
  + \frac{1}{2} {}^{I_k}_{G}\hat{R}^\top (a_{m,k} - \hat b_a)\Delta t^2
  $$

这些公式是标准 IMU 离散积分：
- 角速度去偏置后积分旋转；
- 加速度去偏置、变换到世界后积分速度、位置；
- 重力项 `g` 扣除。

---

## 10. 噪声协方差计算

在 Propagator.cpp 中，离散噪声矩阵由连续噪声转换而来：

```cpp
Qc.block(0,0,3,3) = sigma_w^2 / dt * I
Qc.block(3,3,3,3) = sigma_a^2 / dt * I
Qc.block(6,6,3,3) = sigma_wb^2 * dt * I
Qc.block(9,9,3,3) = sigma_ab^2 * dt * I
Qd = G * Qc * G.transpose();
```

这对应经典 IMU 噪声模型：
- 角速度噪声；
- 线加速度噪声；
- 陀螺偏置漂移；
- 加速度偏置漂移。

---

## 11. 最后将预测结果写回状态

在 `propagate_and_clone(...)` 的尾部：

```cpp
StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_summed, Qd_summed);
state->_timestamp = timestamp;
StateHelper::augment_clone(state, last_w);
```

- `EKFPropagation(...)`：用累积 `Phi_summed` 和 `Qd_summed` 更新状态协方差；
- `augment_clone(...)`：在当前时间点做“克隆”状态，用于后续 MSCKF 视觉更新。

`augment_clone` 也会保存最后一个 `last_w`，用于相机-IMU 时间偏置估计。

---

## 12. 何时 IMU 数据才真正被“使用”

总结一下：
- `callback_inertial()` 只负责把 IMU 塞入缓冲；
- 真正预测发生在：
  - 相机帧到来后；
  - `VioManager::do_feature_propagate_update()` 调用 `propagator->propagate_and_clone(state, message.timestamp)`；
- 这说明系统是“IMU先缓存，图像触发状态推进”。

---

## 13. 其他关联模块

### 13.1 初始化阶段
如果 `is_initialized_vio == false`：
- `initializer->feed_imu(...)` 也会保存 IMU；
- 初始化器 later 通过图像特征和 IMU 数据估计初始状态。

### 13.2 ZUPT（若启用）
如果启用了零速度更新：
- `updaterZUPT->feed_imu(message, oldest_time)` 也会接收 IMU；
- 该模块在检测到静止时，会利用 IMU 计算零速度约束。

---

## 14. 关键结论

run_subscribe_msckf.cpp 的 IMU 处理路径可以拆成两层：
1. ROS 订阅层：`setup_subscribers()` → `callback_inertial()`；
2. VIO 处理层：`VioManager::feed_measurement_imu()` → `Propagator::feed_imu()` → `propagate_and_clone()`。

其中最重要的数学部分在 `Propagator`，它使用标准视觉惯性导航中的：
- IMU 数据时间对齐；
- 偏置校正；
- 离散积分更新；
- EKF 协方差传播。


---

## 1. `select_imu_readings()` - IMU 数据时间对齐与边界插值

### 1.1 函数目的

这个函数的任务是从 IMU 缓存中选择 `[time0, time1]` 时间区间内的 IMU 数据，**并对边界进行插值**，确保获得的 IMU 数据精确对齐相机时间。

### 1.2 时间标定背景

在 `propagate_and_clone()` 中调用时：
```cpp
double time0 = state->_timestamp + last_prop_time_offset;  // 起始时间（IMU 时钟）
double time1 = timestamp + t_off_new;                       // 结束时间（IMU 时钟）
prop_data = Propagator::select_imu_readings(imu_data, time0, time1);
```

其中 `last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0)` 是相机-IMU 时间偏移：
$$t_{IMU} = t_{CAM} + dt_{CAMtoIMU}$$

### 1.3 四种情况处理

代码中逐一遍历 `imu_data` 向量，对每个相邻的 IMU 读数对 `[imu_data[i], imu_data[i+1]]` 判断是否在积分区间内：

#### **情况 1：区间起点插值**（CASE 1）
```cpp
if (imu_data.at(i + 1).timestamp > time0 && imu_data.at(i).timestamp < time0) {
    ov_core::ImuData data = Propagator::interpolate_data(imu_data.at(i), imu_data.at(i + 1), time0);
    prop_data.push_back(data);
}
```

当 IMU 读数跨越 `time0` 时，在两个相邻 IMU 读数之间插值得到恰好在 `time0` 时刻的数据。

**插值公式**（`interpolate_data`）：
```cpp
double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
```

即线性插值：
$$\mathbf{m}(t) = (1-\lambda) \mathbf{m}_i + \lambda \mathbf{m}_{i+1}$$

其中 $\lambda = \frac{t - t_i}{t_{i+1} - t_i}$

#### **情况 2：中间区间数据**（CASE 2）
```cpp
if (imu_data.at(i).timestamp >= time0 && imu_data.at(i + 1).timestamp <= time1) {
    prop_data.push_back(imu_data.at(i));
}
```

当 IMU 读数完全在积分区间 `[time0, time1]` 内时，直接使用该读数。

#### **情况 3：区间末尾插值**（CASE 3）
```cpp
if (imu_data.at(i + 1).timestamp > time1) {
    if (imu_data.at(i).timestamp > time1) {
        ov_core::ImuData data = interpolate_data(imu_data.at(i - 1), imu_data.at(i), time1);
        prop_data.push_back(data);
    } else {
        prop_data.push_back(imu_data.at(i));
        ov_core::ImuData data = interpolate_data(imu_data.at(i), imu_data.at(i + 1), time1);
        prop_data.push_back(data);
    }
    break;
}
```

区间末尾也需要插值确保最后一个数据恰好在 `time1` 时刻。

#### **情况 4：补齐末尾**（CASE 3.4）
```cpp
if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
    ov_core::ImuData data = interpolate_data(imu_data.at(imu_data.size() - 2), imu_data.at(imu_data.size() - 1), time1);
    prop_data.push_back(data);
}
```

如果缓存中没有足够 IMU 数据到达 `time1`，则用最后两个 IMU 读数插值补齐。

### 1.4 数据结构示例

假设原始 IMU 缓存：
```
imu_data[0]: t=0.00, w=[...], a=[...]
imu_data[1]: t=0.01, w=[...], a=[...]
imu_data[2]: t=0.02, w=[...], a=[...]
imu_data[3]: t=0.03, w=[...], a=[...]
```

要求 `time0=0.005, time1=0.028`，则 `select_imu_readings()` 返回：
```
prop_data[0]: t=0.005 ← 情况1插值 imu_data[0] 和 imu_data[1]
prop_data[1]: t=0.01  ← 情况2直接取 imu_data[1]
prop_data[2]: t=0.02  ← 情况2直接取 imu_data[2]
prop_data[3]: t=0.028 ← 情况3插值 imu_data[2] 和 imu_data[3]
```

### 1.5 边界检查

最后还做两个检查：
```cpp
// 确保没有零时间间隔（会导致噪声协方差无穷大）
for (size_t i = 0; i < prop_data.size() - 1; i++) {
    if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) < 1e-12) {
        prop_data.erase(prop_data.begin() + i);
        i--;
    }
}

// 确保至少有 2 个测量点
if (prop_data.size() < 2) {
    return prop_data;  // 返回空或不足长度的向量，触发警告
}
```

---

## 2. `predict_and_compute()` - 状态转移矩阵 F 和离散噪声 Qd

### 2.1 函数入参

```cpp
void Propagator::predict_and_compute(std::shared_ptr<State> state, 
                                     const ov_core::ImuData &data_minus, 
                                     const ov_core::ImuData &data_plus,
                                     Eigen::MatrixXd &F, 
                                     Eigen::MatrixXd &Qd)
```

- `data_minus`, `data_plus`：区间 `[i, i+1]` 的两个 IMU 读数；
- `F`：离散状态转移矩阵（输出）；
- `Qd`：离散过程噪声协方差（输出）。

### 2.2 预处理：IMU 测量校正

#### **步骤 1：去偏置**
```cpp
double dt = data_plus.timestamp - data_minus.timestamp;

Eigen::Vector3d a_hat1 = data_minus.am - state->_imu->bias_a();
Eigen::Vector3d a_hat2 = data_plus.am - state->_imu->bias_a();
```

去掉加速度偏置 $\hat{\mathbf{a}}_i = \mathbf{a}_{m,i} - \hat{\mathbf{b}}_a$

#### **步骤 2：IMU 内参校正**
```cpp
Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
Eigen::Matrix3d R_ACCtoIMU = state->_calib_imu_ACCtoIMU->Rot();
a_hat1 = R_ACCtoIMU * Da * a_hat1;
a_hat2 = R_ACCtoIMU * Da * a_hat2;
```

校正后的加速度：
$$\hat{\mathbf{a}}_{corr} = R_{A \to I} D_a (\mathbf{a}_m - \hat{\mathbf{b}}_a)$$

其中：
- $R_{A \to I}$：加速度计到 IMU 体系的旋转（外参校准）；
- $D_a$：加速度计的 scale/misalignment 矩阵（内参校准）。

#### **步骤 3：陀螺校正（包括重力敏感性）**
```cpp
Eigen::Vector3d w_hat1 = data_minus.wm - state->_imu->bias_g() - Tg * a_hat1;
Eigen::Vector3d w_hat2 = data_plus.wm - state->_imu->bias_g() - Tg * a_hat2;

Eigen::Matrix3d Dw = State::Dm(...);
Eigen::Matrix3d R_GYROtoIMU = state->_calib_imu_GYROtoIMU->Rot();
w_hat1 = R_GYROtoIMU * Dw * w_hat1;
w_hat2 = R_GYROtoIMU * Dw * w_hat2;
```

校正后的角速度：
$$\hat{\boldsymbol{\omega}}_{corr} = R_{G \to I} D_w (\boldsymbol{\omega}_m - \hat{\mathbf{b}}_g - T_g \hat{\mathbf{a}}_{corr})$$

其中 $T_g$ 是重力敏感性矩阵（将加速度项耦合到陀螺）。

### 2.3 选择积分方法

代码支持三种积分方法，根据 `state->_options.integration_method`：

1. **ANALYTICAL**：解析积分（高精度）
2. **RK4**：四阶 Runge-Kutta 积分（平衡精度和速度）
3. **DISCRETE**：离散积分（最快）

以 RK4 为例调用状态预测：
```cpp
predict_mean_rk4(state, dt, w_hat1, a_hat1, w_hat2, a_hat2, new_q, new_v, new_p);
```

### 2.4 离散状态转移矩阵 F 的构建

F 矩阵的典型形式（对于 IMU 状态 + 偏置）：

$$F = \begin{bmatrix}
\partial \theta_{k+1} / \partial \theta_k & \cdots & \partial \theta_{k+1} / \partial \mathbf{b}_g \\
\vdots & \ddots & \vdots \\
\partial \mathbf{b}_{g,k+1} / \partial \theta_k & \cdots & \partial \mathbf{b}_{g,k+1} / \partial \mathbf{b}_g
\end{bmatrix}$$

详细的块结构（来自 `compute_F_and_G_analytic()`）：

```cpp
// 对于旋转状态
F.block(th_id, th_id, 3, 3) = dR_ktok1;
F.block(p_id, th_id, 3, 3) = -skew_x(new_p - p_k - v_k * dt + 0.5 * _gravity * dt * dt) * R_k.transpose();
F.block(v_id, th_id, 3, 3) = -skew_x(new_v - v_k + _gravity * dt) * R_k.transpose();

// 对于位置
F.block(p_id, p_id, 3, 3).setIdentity();

// 对于速度
F.block(p_id, v_id, 3, 3) = Eigen::Matrix3d::Identity() * dt;
F.block(v_id, v_id, 3, 3).setIdentity();

// 对于陀螺偏置
F.block(th_id, bg_id, 3, 3) = -dR_ktok1 * Jr_ktok1 * dt * R_wtoI * Dw;
F.block(p_id, bg_id, 3, 3) = R_k.transpose() * Xi_4 * R_wtoI * Dw;
F.block(v_id, bg_id, 3, 3) = R_k.transpose() * Xi_3 * R_wtoI * Dw;
F.block(bg_id, bg_id, 3, 3).setIdentity();

// 对于加速度偏置
F.block(th_id, ba_id, 3, 3) = dR_ktok1 * Jr_ktok1 * dt * R_wtoI * Dw * Tg * R_atoI * Da;
F.block(p_id, ba_id, 3, 3) = -R_k.transpose() * (Xi_2 + Xi_4 * R_wtoI * Dw * Tg) * R_atoI * Da;
F.block(v_id, ba_id, 3, 3) = -R_k.transpose() * (Xi_1 + Xi_3 * R_wtoI * Dw * Tg) * R_atoI * Da;
F.block(ba_id, ba_id, 3, 3).setIdentity();
```

**关键符号说明**：
- $d R_{k \to k+1}$：两个时刻之间的旋转增量；
- $J_r(\cdot)$：右 Jacobian 矩阵（$Jr\_ktok1$）；
- $\Xi_i$：从解析积分得到的积分矩阵（见 `compute_Xi_sum()`）；
- $\text{skew}(\mathbf{v})$：向量的反对称矩阵。

### 2.5 离散过程噪声协方差 Qd 的构建

首先定义连续时间噪声协方差矩阵 $Q_c$：

```cpp
Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
Qc.block(0, 0, 3, 3) = std::pow(_noises.sigma_w, 2) / dt * Eigen::Matrix3d::Identity();
Qc.block(3, 3, 3, 3) = std::pow(_noises.sigma_a, 2) / dt * Eigen::Matrix3d::Identity();
Qc.block(6, 6, 3, 3) = std::pow(_noises.sigma_wb, 2) / dt * Eigen::Matrix3d::Identity();
Qc.block(9, 9, 3, 3) = std::pow(_noises.sigma_ab, 2) / dt * Eigen::Matrix3d::Identity();
```

对应的噪声模型：
$$Q_c = \text{diag}(\sigma_w^2/dt, \sigma_w^2/dt, \sigma_w^2/dt, \sigma_a^2/dt, \sigma_a^2/dt, \sigma_a^2/dt, \sigma_{wb}^2 dt, \sigma_{wb}^2 dt, \sigma_{wb}^2 dt, \sigma_{ab}^2 dt, \sigma_{ab}^2 dt, \sigma_{ab}^2 dt)$$

其中：
- $\sigma_w$：角速度噪声强度；
- $\sigma_a$：线加速度噪声强度；
- $\sigma_{wb}$：陀螺偏置漂移强度；
- $\sigma_{ab}$：加速度偏置漂移强度。

然后离散化为：

```cpp
Qd = G * Qc * G.transpose();
Qd = 0.5 * (Qd + Qd.transpose());  // 对称化数值误差
```

即：
$$Q_d = G Q_c G^T$$

其中 $G$ 是噪声对状态的影响矩阵，形式类似于 $F$ 但只有在噪声项的列。

### 2.6 最后更新 IMU 状态

```cpp
Eigen::Matrix<double, 16, 1> imu_x = state->_imu->value();
imu_x.block(0, 0, 4, 1) = new_q;
imu_x.block(4, 0, 3, 1) = new_p;
imu_x.block(7, 0, 3, 1) = new_v;
state->_imu->set_value(imu_x);
state->_imu->set_fej(imu_x);  // First Estimate Jacobian（如果启用）
```

注意 `set_fej()` 保存 first-estimate Jacobian，这是某些高级滤波算法（如 OC-EKF）用于提高一致性的技巧。

---

## 3. `StateHelper::augment_clone()` - IMU 克隆与协方差增广

### 3.1 函数目的与调用时机

```cpp
StateHelper::augment_clone(state, last_w);
```

这个函数在每次 IMU 预测完成后调用，目的是**在状态向量中保存当前 IMU 时刻的姿态克隆**，用于后续视觉更新中的 MSCKF 约束。

### 3.2 克隆数据结构

**IMU 状态** `state->_imu`：
```
[q_GtoI (4×1), p_IinG (3×1), v_IinG (3×1), bias_g (3×1), bias_a (3×1)]
   16 维状态
```

**克隆存储**：
```cpp
state->_clones_IMU: map<double (timestamp), shared_ptr<PoseJPL>>
```

这是一个 map，键是时间戳，值是该时刻的姿态（旋转 + 位置）。

### 3.3 克隆过程

#### **步骤 1：检查重复克隆**
```cpp
if (state->_clones_IMU.find(state->_timestamp) != state->_clones_IMU.end()) {
    PRINT_ERROR(...);
    std::exit(EXIT_FAILURE);
}
```

确保不会在同一时刻创建两个克隆（这会导致协方差矩阵退化）。

#### **步骤 2：克隆当前 IMU 姿态**
```cpp
std::shared_ptr<Type> posetemp = StateHelper::clone(state, state->_imu->pose());
std::shared_ptr<PoseJPL> pose = std::dynamic_pointer_cast<PoseJPL>(posetemp);

state->_clones_IMU[state->_timestamp] = pose;
```

`StateHelper::clone()` 做的是：
- 复制当前 IMU 的旋转和位置（6 维）；
- 将其添加到状态协方差矩阵的右下角（增广）；
- 更新所有现有状态与新克隆之间的交叉协方差。

### 3.4 协方差增广

新克隆加入后，协方差矩阵从大小 $n \times n$ 增至 $(n+6) \times (n+6)$：

$$P' = \begin{bmatrix}
P & P_{:, clone} \\
P_{clone,:} & P_{clone, clone}
\end{bmatrix}$$

新的交叉协方差块从已有的关联通过链式法则得到：

```cpp
P_{cross} = P_{old} \cdot F_{clone}^T
```

其中 $F_{clone}$ 是从旧 IMU 到新克隆的状态转移。

### 3.5 时间偏置校准（如果启用）

```cpp
if (state->_options.do_calib_camera_timeoffset) {
    Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
    dnc_dt.block(0, 0, 3, 1) = last_w;  // 最后时刻的角速度
    dnc_dt.block(3, 0, 3, 1) = state->_imu->vel();  // 速度
    
    // 增广协方差
    state->_Cov.block(0, pose->id(), state->_Cov.rows(), 6) +=
        state->_Cov.block(0, state->_calib_dt_CAMtoIMU->id(), state->_Cov.rows(), 1) * dnc_dt.transpose();
    state->_Cov.block(pose->id(), 0, 6, state->_Cov.rows()) +=
        dnc_dt * state->_Cov.block(state->_calib_dt_CAMtoIMU->id(), 0, 1, state->_Cov.rows());
}
```

这是根据 Li & Mourikis 的论文，当估计相机-IMU 时间偏移时，新克隆与时间偏移参数的协方差不是独立的，而是受到 $\frac{\partial \mathbf{p}}{\partial dt}$ 和 $\frac{\partial \theta}{\partial dt}$ 的耦合：

$$\frac{\partial \theta}{\partial dt} \approx \boldsymbol{\omega}  \quad \text{（角速度）}$$
$$\frac{\partial \mathbf{p}}{\partial dt} \approx \mathbf{v}  \quad \text{（线速度）}$$

所以在增广协方差时需要考虑这个关联。

### 3.6 边缘化旧克隆

当克隆数量超过最大值时，最旧的克隆被边缘化：

```cpp
void StateHelper::marginalize_old_clone(std::shared_ptr<State> state) {
    if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
        double marginal_time = state->margtimestep();
        StateHelper::marginalize(state, state->_clones_IMU.at(marginal_time));
        state->_clones_IMU.erase(marginal_time);
    }
}
```

边缘化时：
- 删除该克隆对应的行和列；
- 保留对其他状态的信息（通过 Schur complement）；
- 这是 MSCKF 滑动窗口的核心机制。

### 3.7 克隆窗口示例

假设 `max_clone_size = 5`，时间轴如下：

```
时间:    t=0     t=1     t=2     t=3     t=4     t=5
克隆:    C0      C1      C2      C3      C4      C5
状态:  [IMU]   [C0]    [C1]    [C2]    [C3]    [C4]   [C5]
       ────────────────────────────────────────────────
                                           ↑ 当克隆数>5时
                                        删除C0
```

这个滑动窗口保证了：
1. 足够的历史信息用于视觉三角化；
2. 计算效率（状态维数有限）；
3. 与后端 Bundle Adjustment 的兼容性。

---

## 总结表格

| 函数 | 输入 | 处理 | 输出 |
|------|------|------|------|
| `select_imu_readings()` | 时间区间 `[time0, time1]` + IMU 缓存 | 4 种情况判断 + 线性插值边界 | 精确对齐时间的 IMU 向量 |
| `predict_and_compute()` | 相邻 IMU 读数 + 当前状态 | 去偏置 + 内参校正 + 积分 + 求导 | 状态转移 F + 过程噪声 Qd |
| `augment_clone()` | 当前时刻 + 最后角速度 | 复制 IMU 姿态 + 增广协方差 + 时偏耦合 | 新克隆 + 更新的协方差矩阵 |

