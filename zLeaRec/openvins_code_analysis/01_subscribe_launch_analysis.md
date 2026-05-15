# subscribe.launch 全流程代码解读

## 1. launch 文件整体解读

### 1.1 文件概览

`subscribe.launch` 是 OpenVINS 的 **ROS Topic 订阅模式** 启动文件。该模式通过订阅 ROS 话题（而非直接读取 rosbag）接收 IMU 和相机数据，适用于实时传感器数据流或 rosbag play 场景。

文件路径: `src/open_vins/ov_msckf/launch/subscribe.launch`

### 1.2 参数定义 (行 1-27)

```xml
<arg name="config"      default="euroc_mav" />
<arg name="config_path" default="$(find ov_msckf)/../config/$(arg config)/estimator_config.yaml" />
<arg name="max_cameras" default="2" />
<arg name="use_stereo"  default="true" />
<arg name="bag_start"   default="0" />
<arg name="bag_rate"    default="1" />
<arg name="dataset"     default="V1_01_easy" />
<arg name="dobag"       default="false" />
```

- `config`: 配置文件目录名（如 `euroc_mav`, `tum_vi`, `rpng_aruco`）
- `config_path`: 指向 `estimator_config.yaml` 主配置文件
- `max_cameras`: 最大相机数量（1=单目, 2=双目）
- `use_stereo`: 是否使用双目模式
- `bag_start`: rosbag 播放起始偏移（秒）
- `bag_rate`: rosbag 播放速率
- `dataset`: 数据集名称
- `dobag`: 是否自动播放 rosbag（默认 `false`，通常手动 `rosbag play`）

轨迹保存参数：
```xml
<arg name="dosave"      default="false" />
<arg name="path_est"    default="/tmp/traj_estimate.txt" />
<arg name="path_time"   default="/tmp/traj_timing.txt" />
```

地面真值可视化参数：
```xml
<arg name="dolivetraj"  default="false" />
<arg name="path_gt"     default="$(find ov_data)/$(arg config)/$(arg dataset).txt" />
```

### 1.3 主节点 (行 31-45)

```xml
<node name="ov_msckf" pkg="ov_msckf" type="run_subscribe_msckf" 
      output="screen" clear_params="true" required="true">
```

**逐字段解释**:
- `name="ov_msckf"`: ROS 节点名
- `pkg="ov_msckf"`: 所属 ROS 包
- `type="run_subscribe_msckf"`: 可执行文件名 → 对应 `run_subscribe_msckf.cpp` 编译生成
- `output="screen"`: 标准输出打印到屏幕
- `clear_params="true"`: 启动前清除该节点的私有参数
- `required="true"`: 该节点死亡则终止整个 launch

传递给节点的参数：

| 参数名 | 类型 | 含义 |
|--------|------|------|
| `verbosity` | string | 日志级别 (ALL/DEBUG/INFO/WARNING/ERROR/SILENT) |
| `config_path` | string | 主配置文件路径 |
| `use_stereo` | bool | 双目模式 |
| `max_cameras` | int | 最大相机数 |
| `record_timing_information` | bool | 是否记录时间统计 |
| `record_timing_filepath` | string | 时间统计文件路径 |

### 1.4 辅助节点

**rosbag 播放节点** (行 48-50):
```xml
<group if="$(arg dobag)">
    <node pkg="rosbag" type="play" name="rosbag" 
          args="-d 1 -r $(arg bag_rate) -s $(arg bag_start) $(arg bag)" required="true"/>
</group>
```
- 仅当 `dobag=true` 时启动
- `-d 1`: 延迟1秒开始播放
- `-r`: 播放速率
- `-s`: 起始时间偏移

**轨迹记录节点** (行 53-58):
```xml
<group if="$(arg dosave)">
    <node name="recorder_estimate" pkg="ov_eval" type="pose_to_file" output="screen" required="true">
        <param name="topic"      type="str" value="/ov_msckf/poseimu" />
        <param name="topic_type" type="str" value="PoseWithCovarianceStamped" />
        <param name="output"     type="str" value="$(arg path_est)" />
    </node>
</group>
```
- 订阅 `/ov_msckf/poseimu` 话题
- 将位姿轨迹保存为文本文件

**实时轨迹对齐节点** (行 62-67):
```xml
<group if="$(arg dolivetraj)">
    <node name="live_align_trajectory" pkg="ov_eval" type="live_align_trajectory" output="log" clear_params="true">
        <param name="alignment_type" type="str" value="posyaw" />
        <param name="path_gt"        type="str" value="$(arg path_gt)" />
    </node>
</group>
```
- 实时将估计轨迹与地面真值对齐
- 使用 `posyaw` 对齐方式（仅估计平移和偏航角）

---

## 2. run_subscribe_msckf.cpp 全流程解读

文件路径: `src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp`

### 2.1 全局变量 (行 38-43)

```cpp
std::shared_ptr<VioManager> sys;      // VIO核心管理器（全局）
std::shared_ptr<ROS1Visualizer> viz;  // ROS1可视化器（全局）
```

使用全局智能指针，使得 ROS 回调函数可以访问核心系统。

### 2.2 main() 函数流程

```
main()
  ├── 1. 解析命令行参数, 获取 config_path
  ├── 2. ros::init() 初始化ROS节点
  ├── 3. 创建 NodeHandle, 从参数服务器读取 config_path
  ├── 4. 创建 YamlParser, 加载配置文件
  ├── 5. 设置日志级别 (verbosity)
  ├── 6. 创建 VioManagerOptions → params.print_and_load(parser)
  ├── 7. params.use_multi_threading_subs = true  // 多线程订阅
  ├── 8. sys = std::make_shared<VioManager>(params)  // 创建VIO系统
  ├── 9. viz = std::make_shared<ROS1Visualizer>(nh, sys)  // 创建可视化器
  ├── 10. viz->setup_subscribers(parser)  // 设置ROS订阅者
  ├── 11. ros::AsyncSpinner 启动多线程轮询
  ├── 12. ros::waitForShutdown() 等待关闭
  └── 13. viz->visualize_final() 最终可视化
```

**关键流程详解**:

#### 步骤 6-7: 参数加载
```cpp
VioManagerOptions params;
params.print_and_load(parser);        // 加载所有估计器、噪声、状态、跟踪器参数
params.use_multi_threading_subs = true; // subscribe模式开启多线程
```
`print_and_load(parser)` 调用链：
- `print_and_load_estimator(parser)` → 加载 StateOptions, InertialInitializerOptions
- `print_and_load_trackers(parser)` → 加载追踪器参数 (KLT/Descriptor, 特征点数等)
- `print_and_load_noise(parser)` → 加载 IMU 噪声参数
- `print_and_load_state(parser)` → 加载相机内外参、IMU内参

#### 步骤 8: 创建 VioManager → 见第3节
```cpp
sys = std::make_shared<VioManager>(params);
```

#### 步骤 9: 创建 ROS1Visualizer → 见第4节
```cpp
viz = std::make_shared<ROS1Visualizer>(nh, sys);
```

#### 步骤 10: 设置订阅者 → 见第5节
```cpp
viz->setup_subscribers(parser);
```

---

## 3. VioManager 构造函数深度解析

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp` (行 50-164)

### 3.1 构造函数流程

```
VioManager::VioManager(VioManagerOptions &params_)
  ├── 1. 保存参数副本
  ├── 2. 打印所有参数
  ├── 3. 设置 OpenCV 线程数和随机种子
  ├── 4. 创建 State 对象
  ├── 5. 设置 IMU 内参初始值 (Dw, Da, Tg, 旋转矩阵)
  ├── 6. 设置相机-IMU时间偏移初始值
  ├── 7. 加载每个相机的内参和外参
  ├── 8. (可选) 打开时间统计文件
  ├── 9. 创建特征追踪器 trackFEATS (KLT 或 Descriptor)
  ├── 10. (可选) 创建 Aruco 标签追踪器 trackARUCO
  ├── 11. 创建状态传播器 propagator
  ├── 12. 创建惯性初始化器 initializer
  ├── 13. 创建 MSCKF 更新器 updaterMSCKF
  ├── 14. 创建 SLAM 更新器 updaterSLAM
  └── 15. (可选) 创建零速更新器 updaterZUPT
```

**核心组件解释**:

#### State (状态对象)
存储所有待估计变量的容器，包含：
- `_imu`: IMU 位姿/速度/偏置 (16维: q_GtoI[4], p_IinG[3], v_IinG[3], bg[3], ba[3])
- `_clones_IMU`: 滑动窗口内的历史 IMU 位姿克隆
- `_features_SLAM`: 长期 SLAM 特征点
- `_calib_*`: 各种标定参数（相机内外参、IMU内参、时间偏移等）
- `_timestamp`: 当前状态时间戳

#### Propagator (状态传播器)
负责 IMU 前向传播，将状态从上一时刻预测到当前时刻。

#### InertialInitializer (惯性初始化器)
负责系统初始化：从静止/运动状态估计初始重力方向、速度、IMU偏置。

#### UpdaterMSCKF (MSCKF更新器)
标准 MSCKF 更新：对短周期特征进行三角化→线性化→零空间投影→EKF更新。

#### UpdaterSLAM (SLAM更新器)
对长期特征进行 EKF 更新，包含 delayed initialization 和 anchor change。

#### UpdaterZeroVelocity (零速更新器)
检测机器人静止状态，施加零速度约束进行更新。

---

## 4. ROS1Visualizer 构造函数深度解析

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 38-149)

### 4.1 构造函数流程

```cpp
ROS1Visualizer::ROS1Visualizer(nh, app, sim)  // sim=nullptr for subscribe mode
```

```
ROS1Visualizer 构造函数
  ├── 1. 创建 tf::TransformBroadcaster (TF广播器)
  ├── 2. 创建 image_transport::ImageTransport
  ├── 3. 创建位姿发布者:
  │     - pub_poseimu  → "/poseimu" (PoseWithCovarianceStamped)
  │     - pub_odomimu  → "/odomimu" (Odometry)
  │     - pub_pathimu  → "/pathimu" (Path, 轨迹路径)
  ├── 4. 创建3D点云发布者:
  │     - pub_points_msckf  → MSCKF特征点
  │     - pub_points_slam   → SLAM特征点
  │     - pub_points_aruco  → ARUCO标签点
  │     - pub_points_sim    → 仿真特征点
  ├── 5. 创建跟踪图像发布者:
  │     - it_pub_tracks  → 跟踪历史的可视化图像
  ├── 6. 创建地面真值发布者:
  │     - pub_posegt, pub_pathgt
  ├── 7. 创建回环检测发布者:
  │     - pub_loop_pose, pub_loop_point, pub_loop_extrinsic, pub_loop_intrinsics
  │     - it_pub_loop_img_depth, it_pub_loop_img_depth_color
  ├── 8. 读取 publish_global_to_imu_tf, publish_calibration_tf 参数
  ├── 9. (可选) 加载地面真值文件 (gt_states)
  ├── 10. (可选) 打开状态保存文件
  └── 11. (可选) 启动多线程图像发布线程
```

### 4.2 关键发布者说明

| 话题名 | 类型 | 用途 |
|--------|------|------|
| `/ov_msckf/poseimu` | PoseWithCovarianceStamped | 当前IMU位姿 + 协方差 |
| `/ov_msckf/odomimu` | Odometry | 当前IMU里程计（高频） |
| `/ov_msckf/pathimu` | Path | 历史IMU轨迹 |
| `/ov_msckf/points_msckf` | PointCloud2 | MSCKF 3D特征点 |
| `/ov_msckf/points_slam` | PointCloud2 | SLAM 3D特征点 |
| `/ov_msckf/posegt` | PoseStamped | 地面真值位姿（如有） |
| `/ov_msckf/pathgt` | Path | 地面真值轨迹（如有） |

---

## 5. setup_subscribers() 深度解析

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 151-195)

### 5.1 函数流程

```cpp
void ROS1Visualizer::setup_subscribers(parser)
```

```
setup_subscribers()
  ├── 1. 创建 IMU 订阅者
  │     - 从参数服务器 / yaml 读取 topic_imu (默认 "/imu0")
  │     - sub_imu = _nh->subscribe(topic_imu, 1000, &callback_inertial, this)
  │     - 队列大小 1000
  │
  └── 2. 创建相机订阅者
        ├── if num_cameras == 2 (双目):
        │     - 使用 message_filters::Synchronizer 同步两个相机的图像
        │     - 时间同步策略: ApproximateTime (近似时间, 容差10ms)
        │     - 注册回调: callback_stereo(msg0, msg1, cam_id0=0, cam_id1=1)
        │
        └── else (单目/多相机非同步):
              - 为每个相机创建独立订阅者
              - 注册回调: callback_monocular(msg, cam_id=i)
```

### 5.2 IMU 订阅

```cpp
sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);
```
- 订阅 `topic_imu` 话题
- 队列大小: 1000 (确保不丢失高频IMU数据，IMU通常 100-400Hz)
- 回调函数: `callback_inertial` → **整个系统的核心驱动入口**

### 5.3 双目相机同步订阅

```cpp
auto image_sub0 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic0, 1);
auto image_sub1 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic1, 1);
auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
sync->registerCallback(boost::bind(&ROS1Visualizer::callback_stereo, this, _1, _2, 0, 1));
```

`sync_pol(10)` 表示10毫秒内的近似时间同步窗口。

---

## 6. callback_inertial() — IMU 回调深度解析

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 438-496)

这是 **整个系统的核心处理入口函数**。

### 6.1 函数完整流程

```cpp
void ROS1Visualizer::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg)
```

```
callback_inertial(msg)
  │
  ├── 1. 格式转换: sensor_msgs::Imu → ov_core::ImuData
  │     message.timestamp = msg->header.stamp.toSec()
  │     message.wm = [wx, wy, wz]  // 角速度测量
  │     message.am = [ax, ay, az]  // 线加速度测量
  │
  ├── 2. 喂入VIO系统:
  │     _app->feed_measurement_imu(message)  // 见6.2节
  │
  ├── 3. 高频里程计发布:
  │     visualize_odometry(message.timestamp)  // 见6.3节
  │
  └── 4. 异步更新处理(关键多线程逻辑):
        if (thread_update_running) return;  // 如果正在更新, 跳过
        thread_update_running = true;
        创建线程 {
          lock(camera_queue_mtx);  // 锁定相机队列

          // 检查是否有所有需要的相机的图像
          if (unique_cam_ids.size() == num_unique_cameras) {
            // 计算IMU时间(在相机时间基准下):
            timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);

            // 处理所有时间戳小于当前IMU时间的相机测量:
            while (!camera_queue.empty() && camera_queue[0].timestamp < timestamp_imu_inC) {
              _app->feed_measurement_camera(camera_queue[0]);  // 见第7节
              visualize();  // 完整可视化更新
              camera_queue.pop_front();
            }
          }
          thread_update_running = false;
        }
        // 根据多线程设置选择 join 或 detach
```

### 6.2 feed_measurement_imu() 深度解析

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp` (行 166-189)

```cpp
void VioManager::feed_measurement_imu(const ov_core::ImuData &message)
```

```
feed_measurement_imu(message)
  ├── 1. 确定最老需要的IMU时间:
  │     - 已初始化: oldest_time = state->margtimestep()
  │     - 未初始化: oldest_time = 当前时间 - 初始化窗口 - 0.10s
  │
  ├── 2. propagator->feed_imu(message, oldest_time)
  │     - 将IMU数据存入内部buffer
  │     - 清理早于 oldest_time 的旧数据
  │
  ├── 3. (未初始化时) initializer->feed_imu(message, oldest_time)
  │     - 将IMU数据传给惯性初始化器
  │
  └── 4. (可选) updaterZUPT->feed_imu(message, oldest_time)
        - 将IMU数据传给零速检测器
```

### 6.3 visualize_odometry() — 高频里程计发布

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 245-350)

```cpp
void ROS1Visualizer::visualize_odometry(double timestamp)
```

```
visualize_odometry(timestamp)
  ├── 1. 若未初始化, 直接返回
  │
  ├── 2. 快速状态传播:
  │     propagator->fast_state_propagate(state, timestamp, state_plus, cov_plus)
  │     - 获取从上次状态时间到当前IMU时间的传播结果
  │     - 返回 13维状态 和 12x12协方差
  │     -   state_plus: [q_GtoI(4), p_IinG(3), v_IinI_local(3), w_IinI(3)]
  │     -   cov_plus:  [q(3x3), p(3x3), v_local(3x3), w(3x3)]
  │
  ├── 3. 发布 Odometry 消息 (如果需要):
  │     odomIinM.pose.pose.orientation = q_GtoI
  │     odomIinM.pose.pose.position = p_IinG
  │     odomIinM.twist.twist.linear = v_IinI_local
  │     odomIinM.twist.twist.angular = w_IinI
  │     // ROS约定: 协方差顺序为 [p, q, v, w]
  │     pub_odomimu.publish(odomIinM)
  │
  └── 4. 发布 TF 变换:
        - "global" → "imu": 当前IMU到全局的变换
        - "imu" → "cam0/1": IMU到各相机的标定外参
```

---

## 7. callback_stereo() — 双目相机回调

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 537-589)

```cpp
void ROS1Visualizer::callback_stereo(msg0, msg1, cam_id0, cam_id1)
```

```
callback_stereo()
  ├── 1. 检查是否需要丢弃(帧率控制):
  │     if timestamp < camera_last_timestamp[cam_id0] + 1.0/track_frequency → return
  │     (默认 track_frequency=20Hz, 则最小帧间隔0.05s)
  │
  ├── 2. cv_bridge 转换 sensor_msgs::Image → cv::Mat (MONO8格式)
  │
  ├── 3. 构建 CameraData 消息:
  │     message.timestamp = 图像时间戳
  │     message.sensor_ids = [cam_id0, cam_id1]
  │     message.images = [img0, img1]
  │     message.masks = [mask0, mask1]  (特征提取掩码)
  │
  └── 4. 加入相机队列:
        lock(camera_queue_mtx);
        camera_queue.push_back(message);
        sort(camera_queue);  // 按时间排序
```

**关键设计**: 相机数据**不直接**处理，而是放入 `camera_queue` 等待。实际的处理在 `callback_inertial()` 中被触发——只有当有 IMU 数据时间戳超过相机数据时，才取出相机数据进行处理。这保证了 IMU 和相机数据的正确时间顺序。

---

## 8. track_image_and_update() — 图像追踪与更新

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp` (行 256-321)

```cpp
void VioManager::track_image_and_update(const CameraData &message_const)
```

```
track_image_and_update(message)
  ├── 1. 数据校验 (sensor_ids 和 images 数量一致)
  │
  ├── 2. (可选) 降采样: cv::pyrDown 将图像缩小一半
  │
  ├── 3. 特征追踪:
  │     trackFEATS->feed_new_camera(message)
  │     - KLT追踪: 从前一帧追踪已有特征点, 检测新特征点
  │     - 或 Descriptor匹配: 通过描述子匹配
  │
  ├── 4. (可选) ARUCO标签检测:
  │     trackARUCO->feed_new_camera(message)
  │
  ├── 5. (可选) 零速更新检测:
  │     updaterZUPT->try_update(state, timestamp)
  │     - 如果检测到静止, 直接施加零速约束并返回
  │
  ├── 6. 初始化检查:
  │     if (!is_initialized_vio) {
  │         is_initialized_vio = try_to_initialize(message);  // 尝试初始化
  │         if (!is_initialized_vio) return;
  │     }
  │
  └── 7. 核心处理:
        do_feature_propagate_update(message)  // 见第9节
```

---

## 9. do_feature_propagate_update() — 核心EKF更新流程

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp` (行 323-714)

这是 OpenVINS **最核心的函数**，包含完整的 MSCKF + SLAM 更新流程。

### 9.1 完整流程

```
do_feature_propagate_update(message)
  │
  ├── [阶段1: 状态传播] (行 330-360)
  │   ├── 检查时间顺序 (防止乱序)
  │   ├── propagator->propagate_and_clone(state, message.timestamp)
  │   │   - IMU积分: 从 state->_timestamp 传播到 message.timestamp
  │   │   - 协方差传播: P' = F·P·F^T + G·Q·G^T
  │   │   - 随机克隆: 在传播终点创建一个新的IMU位姿克隆
  │   └── 检查是否有足够的克隆 (至少5个)
  │
  ├── [阶段2: 特征分类] (行 366-500)
  │   ├── 获取三类特征:
  │   │   - feats_lost: 在当前帧丢失的特征 → MSCKF更新
  │   │   - feats_marg: 包含最老克隆的特征 → MSCKF更新或SLAM
  │   │   - feats_slam: 已有的SLAM特征
  │   │
  │   ├── 特征数检查/去重
  │   ├── 检测超长跟踪特征 → 转为SLAM特征
  │   └── SLAM特征管理: 标记丢失的SLAM特征为待边缘化
  │
  ├── [阶段3: MSCKF更新] (行 520-527)
  │   └── updaterMSCKF->update(state, featsup_MSCKF)
  │       - 对每个特征: 三角化 → 线性化 → 零空间投影
  │       - 堆叠所有特征的压缩残差
  │       - EKF更新: K = P·H^T·(H·P·H^T + R)^{-1}
  │
  ├── [阶段4: SLAM更新] (行 532-548)
  │   ├── updaterSLAM->update(state, feats_slam_UPDATE)
  │   │   - 对已知SLAM特征进行标准EKF更新
  │   └── updaterSLAM->delayed_init(state, feats_slam_DELAYED)
  │       - 对新的SLAM特征进行延迟初始化 (三角化后加入状态)
  │
  ├── [阶段5: 清理] (行 554-597)
  │   ├── 重三角化当前活跃特征 (用于可视化)
  │   ├── 清理特征数据库中的已使用特征
  │   ├── Anchor change (SLAM特征的锚点切换)
  │   ├── 清理早于边缘化时间戳的测量
  │   └── StateHelper::marginalize_old_clone(state)
  │       - 边缘化最老的IMU位姿克隆
  │
  └── [阶段6: 统计输出] (行 599-714)
      ├── 打印各阶段耗时
      ├── (可选) 写入时间统计文件
      ├── 更新累计距离
      └── 打印当前状态 (位姿/速度/偏置/标定参数)
```

### 9.2 特征分类逻辑细节

```
特征按状态分类:
  ┌──────────────────────┐
  │  所有被追踪的特征      │
  └──────────────────────┘
           │
           ▼
  ┌──────────────────────────────────────┐
  │ feats_lost: 当前帧不再观测到的特征      │ → MSCKF更新
  │ feats_marg: 包含最老克隆时间戳的特征    │ → MSCKF更新 (用于边缘化)
  │ feats_maxtracks: 跟踪超长的特征        │ → 转为SLAM特征
  │ feats_slam: 已存在的SLAM特征           │ → SLAM更新
  └──────────────────────────────────────┘
```

**MSCKF核心思想**: 不在状态中显式估计特征点的3D位置，而是通过零空间投影消除特征位置的不确定性，将视觉约束压缩为仅与机器人状态相关的残差。

**SLAM特征**: 将长期跟踪的特征显式地加入状态向量中估计（3D位置或反向深度），提供长期的视觉约束。

---

## 10. visualize() — 完整可视化

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 197-243)

```cpp
void ROS1Visualizer::visualize()
```

```
visualize()
  ├── 检查是否已在此时刻可视化过
  ├── publish_images()          → 发布跟踪历史图像
  ├── 若未初始化, return
  ├── publish_state()           → 发布 IMU 位姿 + 路径
  │     - pub_poseimu: PoseWithCovarianceStamped
  │     - pub_pathimu: Path (降采样以避免rviz崩溃)
  ├── publish_features()        → 发布 3D 特征点云
  ├── publish_groundtruth()     → 发布地面真值并计算误差
  └── publish_loopclosure_information()  → 发布回环检测相关信息
```

### 10.1 publish_groundtruth() — 误差计算

文件路径: `src/open_vins/ov_msckf/src/ros/ROS1Visualizer.cpp` (行 711-838)

实时计算并输出以下指标：

```
误差计算:
  ├── 位置误差: err_pos = ||p_est - p_gt||
  ├── 姿态误差: err_ori = 2 * ||quat_multiply(q_est, Inv(q_gt))[:3]|| * (180/π)
  ├── 累计 RMSE:
  │     summed_mse_ori += err_ori²
  │     summed_mse_pos += err_pos²
  │     RMSE = sqrt(summed / count)
  ├── NEES (归一化估计误差平方):
  │     nees_ori = (2*quat_diff_vec)^T · cov_ori^{-1} · (2*quat_diff_vec)
  │     nees_pos = (p_est - p_gt)^T · cov_pos^{-1} · (p_est - p_gt)
  └── 输出: error, rmse, nees (每个timestamp)
```

---

## 11. 完整数据流总结

```
sensor_msgs::Imu (400Hz)                 sensor_msgs::Image (20Hz, stereo)
        │                                         │
        ▼                                         ▼
callback_inertial()                       callback_stereo()
  │                                         │
  ├─ feed_measurement_imu()                ├─ 转换为 CameraData
  │   ├─ propagator->feed_imu()            ├─ 加入 camera_queue
  │   ├─ initializer->feed_imu()           └─ 等待IMU触发
  │   └─ zupt->feed_imu()
  │
  ├─ visualize_odometry() (高频发布)
  │   └─ fast_state_propagate() → pub_odomimu
  │
  └─ 异步线程:
        while (camera_queue[0].timestamp < imu_timestamp_in_camera_frame) {
          │
          feed_measurement_camera(camera_queue[0])
            └─ track_image_and_update()
                 ├─ trackFEATS->feed_new_camera()  // 特征追踪
                 ├─ try_to_initialize()             // (首次) 惯性初始化
                 └─ do_feature_propagate_update()
                      ├─ propagator->propagate_and_clone()  // IMU传播 + 克隆
                      ├─ updaterMSCKF->update()              // MSCKF EKF更新
                      ├─ updaterSLAM->update()               // SLAM EKF更新
                      ├─ updaterSLAM->delayed_init()         // SLAM延迟初始化
                      └─ marginalize_old_clone()             // 边缘化最老克隆
          │
          visualize()
            ├─ publish_state()          → /poseimu (带协方差)
            ├─ publish_features()       → /points_msckf, /points_slam
            ├─ publish_groundtruth()    → RMSE, NEES 实时计算
            └─ publish_loopclosure()    → 回环检测数据
        }
```

## 12. 关键线程模型

subscribe 模式使用 **多线程异步模型**:

```
主线程 (ROS AsyncSpinner):
  ├─ IMU回调 (高频, ~400Hz):
  │   ├─ 喂入IMU数据 (快速, 仅buffer操作)
  │   ├─ 高频里程计发布
  │   └─ 触发异步更新线程 (如果前次更新已完成)
  │
  └─ 相机回调 (~20Hz):
      └─ 加入camera_queue (仅入队)

工作线程 (异步):
  └─ 从camera_queue取数据, 执行完整的 propagation → update → marginalization
```

**关键同步机制**:
- `camera_queue_mtx`: 互斥锁保护相机队列
- `thread_update_running`: 原子布尔值防止多个更新线程同时运行
- `use_multi_threading_subs=true` 时更新线程 detach，否则 join（阻塞IMU回调）
