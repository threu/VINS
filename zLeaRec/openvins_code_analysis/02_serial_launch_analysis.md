# serial.launch 全流程代码解读

## 1. launch 文件整体解读

### 1.1 文件概览

`serial.launch` 是 OpenVINS 的 **ROS Bag 串行读取模式** 启动文件。与 subscribe 模式不同，该模式直接在 C++ 代码中打开并顺序读取 rosbag 文件，**不依赖** ROS 话题订阅机制来获取数据。

文件路径: `src/open_vins/ov_msckf/launch/serial.launch`

### 1.2 与 subscribe.launch 的关键差异

| 特性 | subscribe.launch | serial.launch |
|------|-----------------|---------------|
| 数据来源 | ROS 话题订阅 | rosbag 直接读取 |
| 双目同步 | message_filters::Synchronizer | 手动查找最近时间戳 |
| 多线程 | AsyncSpinner + 异步更新 | 单线程串行处理 |
| 时钟源 | ROS 时间 (rosbag play 提供) | rosbag 消息时间戳 |
| 适用场景 | 实时传感器 / rosbag play | 离线评测、精确复现 |

### 1.3 参数定义 (行 1-27)

```xml
<arg name="config"      default="tum_vi" />
<arg name="bag"         default="/home/patrick/datasets/$(arg config)/$(arg dataset).bag" />
<arg name="bag_start"   default="0" />
```

注意：serial.launch **不包含** `dobag` 参数和 rosbag play 节点，因为 rosbag 在 C++ 代码中直接读取。

其他参数（verbosity, config_path, use_stereo, max_cameras, dosave, path_est 等）与 subscribe.launch 基本一致。

### 1.4 主节点 (行 29-48)

```xml
<node name="ov_msckf" pkg="ov_msckf" type="ros1_serial_msckf" 
      output="screen" clear_params="true" required="true">
```

**关键参数**（与 subscribe 模式不同的部分）:

| 参数名 | 类型 | 含义 |
|--------|------|------|
| `path_bag` | string | rosbag 文件完整路径 |
| `bag_start` | double | rosbag 起始时间偏移（秒） |
| `bag_durr` | int | rosbag 播放时长（秒），-1 表示全部 |
| `topic_imu` | string | rosbag 中的 IMU 话题名 |
| `topic_camera0/1/...` | string | rosbag 中的相机话题名 |
| `path_gt` | string | (可选) 地面真值文件路径 |

### 1.5 辅助节点

- `recorder_estimate`: 与 subscribe 模式相同，记录位姿轨迹
- `live_align_trajectory`: 与 subscribe 模式相同，实时对齐地面真值

---

## 2. ros1_serial_msckf.cpp 全流程解读

文件路径: `src/open_vins/ov_msckf/src/ros1_serial_msckf.cpp`

### 2.1 全局变量 (行 37-38)

```cpp
std::shared_ptr<VioManager> sys;      // VIO核心管理器
std::shared_ptr<ROS1Visualizer> viz;  // ROS1可视化器
```

与 subscribe 模式使用完全相同的核心类和可视化器。

### 2.2 main() 函数总流程

```
main()
  ├── [阶段1: 初始化] (行 44-77)
  │   ├── 解析命令行/config_path
  │   ├── ros::init + NodeHandle
  │   ├── 创建 YamlParser + 加载配置
  │   ├── 创建 VioManager
  │   │   ├── VioManagerOptions params
  │   │   ├── params.print_and_load(parser)
  │   │   ├── params.use_multi_threading_subs = false  // ← 关键: 单线程
  │   │   └── sys = make_shared<VioManager>(params)
  │   └── viz = make_shared<ROS1Visualizer>(nh, sys)
  │
  ├── [阶段2: 读取 rosbag 参数] (行 82-121)
  │   ├── 获取 IMU 话题名 (topic_imu)
  │   ├── 获取相机话题名 (topic_camera0, topic_camera1, ...)
  │   ├── 获取 rosbag 路径 (path_bag)
  │   ├── (可选) 加载地面真值 (gt_states)
  │   └── 获取起始时间和持续时间 (bag_start, bag_durr)
  │
  ├── [阶段3: 建立消息索引] (行 128-185)
  │   ├── 打开 rosbag 文件
  │   ├── 创建 rosbag::View (按时间窗口)
  │   └── 遍历全部消息, 收集 IMU 和相机消息索引
  │       - 按原始顺序保存到 msgs vector
  │       - 记录最后一个相机消息的时间 (max_camera_time)
  │
  └── [阶段4: 串行逐条处理] (行 192-276)
      └── for each msg in msgs:
            ├── IMU消息 → viz->callback_inertial()
            └── 相机消息 → 同步配对 → viz->callback_stereo() 或 callback_monocular()
```

### 2.3 阶段2: 读取话题参数

```cpp
// IMU 话题
std::string topic_imu;
nh->param<std::string>("topic_imu", topic_imu, "/imu0");
parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);

// 相机话题 (支持多相机)
std::vector<std::string> topic_cameras;
for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic;
    nh->param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
}
```

话题名可以从两个来源获取：
1. ROS 参数服务器（优先级最高）
2. YAML 配置文件中的 `relative_config_imucam` 段

### 2.4 阶段3: 建立消息索引

```cpp
rosbag::Bag bag;
bag.open(path_to_bag, rosbag::bagmode::Read);

rosbag::View view_full;
view_full.addQuery(bag);
ros::Time time_init = view_full.getBeginTime() + ros::Duration(bag_start);
ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);

rosbag::View view;
view.addQuery(bag, time_init, time_finish);
```

然后遍历 view 中的所有消息，建立索引：

```cpp
std::vector<rosbag::MessageInstance> msgs;
for (const rosbag::MessageInstance &msg : view) {
    if (msg.getTopic() == topic_imu) {
        msgs.push_back(msg);  // 记录IMU消息
    }
    for (int i = 0; i < params.state_options.num_cameras; i++) {
        if (msg.getTopic() == topic_cameras.at(i)) {
            msgs.push_back(msg);  // 记录相机消息
            max_camera_time = std::max(max_camera_time, msg.getTime().toSec());
        }
    }
}
```

**关键设计**: 不 instantiate 消息体（即不实际反序列化），仅通过检查 topic 名称建立索引。这大大加快了 rosbag 扫描速度。

### 2.5 阶段4: 串行处理循环

这是 serial 模式**最核心的不同之处**——双目同步逻辑：

```cpp
for (int m = 0; m < (int)msgs.size(); m++) {
    // IMU 处理
    if (msgs.at(m).getTopic() == topic_imu) {
        viz->callback_inertial(msgs.at(m).instantiate<sensor_msgs::Imu>());
        // 注意: 这里直接调用 callbac_inertial, 不经过ROS话题系统!
    }

    // 相机处理
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {
        if (msgs.at(m).getTopic() != topic_cameras.at(cam_id))
            continue;

        // ===== 手动双目同步 =====
        // 寻找当前时间戳 0.02s 内的其他相机消息
        std::map<int, int> camid_to_msg_index;
        double meas_time = msgs.at(m).getTime().toSec();
        for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
            if (cam_idt == cam_id) {
                camid_to_msg_index.insert({cam_id, m});
                continue;
            }
            // 向前搜索最近匹配
            int cam_idt_idx = -1;
            for (int mt = m; mt < (int)msgs.size(); mt++) {
                if (msgs.at(mt).getTopic() != topic_cameras.at(cam_idt))
                    continue;
                if (std::abs(msgs.at(mt).getTime().toSec() - meas_time) < 0.02)
                    cam_idt_idx = mt;
                break;
            }
            if (cam_idt_idx != -1) {
                camid_to_msg_index.insert({cam_idt, cam_idt_idx});
            }
        }

        // 若未找到全部相机 → 跳过
        if (camid_to_msg_index.size() != params.state_options.num_cameras)
            continue;

        // (可选) Groundtruth 初始化
        if (!gt_states.empty() && !sys->initialized()) {
            sys->initialize_with_gt(imustate);
        }

        // 喂入相机数据
        viz->callback_stereo(msg0, msg1, 0, 1);
        // 标记已处理的消息
        used_index.insert(camid_to_msg_index.at(0));
        used_index.insert(camid_to_msg_index.at(1));

        break;  // 跳出相机循环
    }
}
```

### 2.6 serial 模式的双目同步策略

与 subscribe 模式的 `message_filters::Synchronizer` 不同，serial 模式手动实现同步：

```
同步窗口: 0.02 秒 (20ms)

消息流:
  time: 1.000  1.001  1.002  1.003  1.004 ...
  cam0:  [x]                  [x]
  cam1:         [x]                  [x]
  IMU:    [x]  [x]    [x]    [x]    [x]   ...

处理逻辑:
  当遇到 cam0@1.000 → 搜索 cam1 在 [1.000, 1.020] 范围内
                    → 找到 cam1@1.001 → 配对成功!
                    → 调用 callback_stereo(cam0@1.000, cam1@1.001)
                    → 标记两条消息为已使用

  当遇到 cam1@1.001 → 已在 used_index 中 → 跳过
```

### 2.7 serial vs subscribe 模式: 回调函数调用路径对比

```
subscribe 模式:
  ROS Spinner → callback_inertial(msg) → 发布者/订阅者回调
              → callback_stereo(msg0, msg1) → 放入camera_queue → 由IMU触发处理

serial 模式:
  main() 循环 → callback_inertial(msg)  (直接调用)
              → callback_stereo(msg0, msg1) (直接调用)
```

**关键差异**: serial 模式下，callback_inertial 和 callback_stereo 都在主线程中**同步直接调用**，不经过 ROS 回调队列。

但这意味着在 serial 模式下：
- `use_multi_threading_subs = false` → callback_inertial 中的异步更新线程会 **join**（阻塞等待）
- 处理流程变为**完全串行**: IMU → 等待相机队列处理完成 → 下一个IMU

### 2.8 Groundtruth 初始化

serial 模式支持从地面真值文件初始化，这在进行算法评估时非常有用：

```cpp
if (!gt_states.empty() && !sys->initialized() && 
    ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
    sys->initialize_with_gt(imustate);
}
```

地面真值状态格式：`[time(s), q_GtoI(4), p_IinG(3), v_IinG(3), b_gyro(3), b_accel(3)]`

---

## 3. initialize_with_gt() 深度解析

文件路径: `src/open_vins/ov_msckf/src/core/VioManager.cpp`

```cpp
void VioManager::initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate)
```

```
initialize_with_gt(imustate)
  ├── 1. 设置 IMU 状态:
  │     state->_imu->quat() = imustate[1:4]    // q_GtoI (JPL四元数)
  │     state->_imu->pos()  = imustate[5:7]    // p_IinG
  │     state->_imu->vel()  = imustate[8:10]   // v_IinG
  │     state->_imu->bias_g() = imustate[11:13] // 陀螺仪偏置
  │     state->_imu->bias_a() = imustate[14:16] // 加速度计偏置
  │
  ├── 2. 设置时间戳:
  │     state->_timestamp = imustate[0] // 秒
  │
  ├── 3. 设置协方差:
  │     位置/姿态: 小初始协方差 (地面真值初始化)
  │     速度/偏置: 中等初始协方差
  │
  └── 4. 设置标志:
        is_initialized_vio = true
        startup_time = imustate[0]
```

---

## 4. 完整数据流总结

```
┌─────────────────────────────────────────────────────────┐
│                serial.launch 启动                       │
│  roslaunch ov_msckf serial.launch config:=euroc_mav     │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│            ros1_serial_msckf.cpp main()                 │
│                                                         │
│  1. 初始化 ROS 节点                                      │
│  2. 创建 VioManager + ROS1Visualizer                    │
│  3. 打开 rosbag                                         │
│  4. 建立消息索引 (不反序列化)                               │
│  5. 逐条串行处理:                                         │
│                                                         │
│     for msg in bag:                                     │
│       if IMU:                                           │
│         callback_inertial() ─────────────────────┐       │
│           ├─ feed_measurement_imu()              │       │
│           ├─ visualize_odometry()                │       │
│           └─ (join) 同步处理camera_queue          │       │
│       if CAM:                                     │       │
│         手动同步双目 → callback_stereo()          │       │
│           └─ camera_queue.push_back()             │       │
│                                                   │       │
│  6. visualize_final() → 输出RMSE/NEES             │       │
└───────────────────────────────────────────────────┘       │
                                                           │
     ┌─────────────────────────────────────────────────────┘
     │
     ▼
  ┌──────────────────────────────────────────────┐
  │ 与 subscribe 模式共享的核心处理流程:             │
  │                                              │
  │  camera_queue → feed_measurement_camera()    │
  │    └─ track_image_and_update()               │
  │         ├─ trackFEATS->feed_new_camera()     │
  │         ├─ try_to_initialize()               │
  │         └─ do_feature_propagate_update()     │
  │              ├─ propagate_and_clone()        │
  │              ├─ updaterMSCKF->update()       │
  │              ├─ updaterSLAM->update()        │
  │              ├─ updaterSLAM->delayed_init()  │
  │              └─ marginalize_old_clone()      │
  └──────────────────────────────────────────────┘
```

## 5. serial 模式的特点与适用场景

### 5.1 优点
- **可复现**: 完全单线程串行执行，每次运行产生完全相同的结果
- **简单**: 不需要 rosbag play 进程，一个 launch 文件搞定
- **高效**: 不经过 ROS 序列化/反序列化（IMU消息除外），减少了通信开销
- **时间可控**: 支持 bag_start、bag_durr 参数，方便评估特定时间段

### 5.2 缺点
- 不支持非 rosbag 数据源（如实时相机）
- 双目同步窗口固定 0.02s，可能丢失某些边缘情况的数据
- 单线程模式处理速度较慢（虽然对离线评测通常不是问题）

### 5.3 vs subscribe 模式选择建议
- **serial**: 离线数据集评测、参数调优（需要可复现结果）、论文实验
- **subscribe**: 实时系统、在线演示、需要灵活数据源切换
