```cpp
sys = std::make_shared<VioManager>(params);
```
```cpp
VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }
}
```
We define the quaternion as the following linear combination:
$$
   \bar{q} = q_4+q_1\mathbf{i}+q_2\mathbf{j}+q_3\mathbf{k}
$$
  Where i,j,k are defined as the following:
 $$
   \mathbf{i}^2=-1~,~\mathbf{j}^2=-1~,~\mathbf{k}^2=-1
 $$
 $$
   -\mathbf{i}\mathbf{j}=\mathbf{j}\mathbf{i}=\mathbf{k}
   ~,~
   -\mathbf{j}\mathbf{k}=\mathbf{k}\mathbf{j}=\mathbf{i}
   ~,~
   -\mathbf{k}\mathbf{i}=\mathbf{i}\mathbf{k}=\mathbf{j}
 $$
  As noted in @cite Trawny2005TR this does not correspond to the Hamilton notation, and follows the "JPL Proposed Standard Conventions".
  The q_4 quantity is the "scalar" portion of the quaternion, while q_1, q_2, q_3 are part of the "vector" portion.
  We split the 4x1 vector into the following convention:
 $$
   \bar{q} = \begin{bmatrix}q_1\\q_2\\q_3\\q_4\end{bmatrix} = \begin{bmatrix}\mathbf{q}\\q_4\end{bmatrix}
 $$
  It is also important to note that the quaternion is constrained to the unit circle:
 $$
   |\bar{q}| = \sqrt{\bar{q}^\top\bar{q}} = \sqrt{|\mathbf{q}|^2+q_4^2} = 1
 $$
 
 
  @section jplquat_errorstate Error State Definition
 
  It is important to note that one can prove that the left-multiplicative quaternion error is equivalent to the SO(3) error.
  If one wishes to use the right-hand error, this would need to be implemented as a different type then this class!
  This is noted in Eq. (71) in @cite Trawny2005TR .
  Specifically we have the following:
  $$
  \begin{align*}
  {}^{I}_G\bar{q} &\simeq \begin{bmatrix} \frac{1}{2} \delta \boldsymbol{\theta} \\ 1 \end{bmatrix} \otimes {}^{I}_G\hat{\bar{q}}
  \end{align*}
  $$
  which is the same as:
  $$
  \begin{align*}
  {}^{I}_G \mathbf{R} &\simeq \exp(-\delta \boldsymbol{\theta}) {}^{I}_G \hat{\mathbf{R}} \\
  &\simeq (\mathbf{I} - \lfloor \delta \boldsymbol{\theta} \rfloor) {}^{I}_G \hat{\mathbf{R}} \\
  \end{align*}
  $$

 
   @brief Implements update operation by left-multiplying the current
   quaternion with a quaternion built from a small axis-angle perturbation.
   
$$
\bar{q}=norm\Big(\begin{bmatrix} \frac{1}{2} \delta \boldsymbol{\theta}_{dx} \\ 1 \end{bmatrix}\Big) \otimes \hat{\bar{q}}
$$
   
 @param dx Axis-angle representation of the perturbing quaternion
  

## 1. 这个工程里的四元数是什么类型

`PoseJPL` 里用的是 `JPLQuat`，也就是 JPL 约定的四元数。

文件位置：
- PoseJPL.h
- JPLQuat.h
- quat_ops.h

`PoseJPL` 内部把 orientation 存成：
- 四元数向量：`[q1, q2, q3, q4]^T`
- 其中 `q4` 是标量部分，`q1,q2,q3` 是向量部分

---

## 2. 与“普通四元数”最大的不同

### 2.1 书写顺序不同
这个工程里用的是 JPL 顺序：
- `q = q4 + q1 i + q2 j + q3 k`
- 向量形式：`[q1, q2, q3, q4]^T`

而很多“普通四元数”实现（尤其是 Hamilton 约定）通常是：
- `q = q0 + q1 i + q2 j + q3 k`
- 向量形式：`[q0, q1, q2, q3]^T`
- 也就是 scalar-first，比如 `[w,x,y,z]`

所以要注意：
- JPL：`[x,y,z,w]`
- 常见 Hamilton：`[w,x,y,z]`

### 2.2 运算规则也不同
JPL 约定下，乘法、旋转矩阵转换公式和 Hamilton 形式不一样。

quat_ops.h 中的乘法是：
- `quat_multiply(q, p)` 计算 `q ⊗ p`
- 并且用了 JPL 的左乘矩阵形式

旋转矩阵公式是：
- `R = (2 q4^2 - 1) I - 2 q4 [q]_x + 2 q q^T`
- 这里的 `q` 是前三个分量，`q4` 是最后一个分量

---

## 3. 这个工程中四元数的实际计算流程

### 3.1 `PoseJPL` 更新方式
`PoseJPL::update()` 里：
- 把前 3 个误差 `dx[0:3]` 视为小角度增量
- 构造扰动四元数 `dq = normalize([0.5*dx; 1])`
- 用左乘更新：
  - `q_new = dq ⊗ q_old`

代码关键点：
- `dq << .5 * dx.block(0, 0, 3, 1), 1.0;`
- `newX.block(0,0,4,1) = ov_core::quat_multiply(dq, quat());`

这说明：
- 该系统采用“左乘误差”模型
- 不是 `q_new = q_old ⊗ dq`
- 这是 JPL/Kalman 误差状态常见的写法

### 3.2 归一化和唯一性
`quatnorm()` 会：
- 如果 `q4 < 0`，把四元数反号
- 再归一化
这保证了四元数在全局上唯一（正的标量部分）

### 3.3 旋转矩阵转换
核心函数：
- `quat_2_Rot(q)`：JPL 四元数转 SO(3)
- `rot_2_quat(R)`：SO(3) 转 JPL 四元数

这两个函数都按 JPL `[x,y,z,w]` 顺序实现。

---

## 4. 工程里其他地方的 JPL 四元数使用

### 4.1 状态变量
- `PoseJPL` 里的 `_q` 是 `JPLQuat`
- `PoseJPL` 中 position 与 orientation 分离

### 4.2 Ceres 参数化
- `src/open_vins/ov_init/src/ceres/State_JPLQuatLocal.h/.cpp`
- 这里也实现了同样的 JPL 四元数局部参数化
- `Plus()` 里也是 `q <- [0.5*dx;1] ⊗ q`

### 4.3 ROS 发布
ROS2Visualizer.cpp 中明确写了：
- “因为我们用的是 JPL，所以发布时要隐式转换到 Hamilton”
- 说明内部储存 JPL，但外部 ROS/TF 可能期望 Hamilton 形式

### 4.4 推进/积分
Propagator.cpp
- 通过 `Omega(w)` 计算四元数导数
- 也使用 JPL 的四元数积分公式和 `quatnorm`

---

## 5. 结论

### 你要记住的关键差异
- 这个工程里的四元数是 JPL 约定
- 储存顺序是 `[x,y,z,w]`，标量在最后
- 采用“左乘误差”更新：`dq ⊗ q`
- quat_ops.h 中的所有旋转、乘法、求逆、归一化都基于这一约定

### 对比普通 Hamilton 四元数
如果你平时习惯的是 `Eigen::Quaterniond` 或 ROS TF 的 `[w,x,y,z]`：
- 一定要注意顺序
- 注意乘法方向
- 注意旋转矩阵转换公式是否匹配

---

## 1. `State` 中设置了哪些参数

### 1.1 核心状态变量
State.h 中的 `State` 主要包含：

- `_imu`：当前活动 IMU 状态，类型 `ov_type::IMU`
- `_clones_IMU`：滑动窗口中的历史 IMU 姿态克隆，`std::map<double, std::shared_ptr<ov_type::PoseJPL>>`
- `_features_SLAM`：当前 SLAM 特征集合
- `_calib_dt_CAMtoIMU`：相机到 IMU 的时间偏移
- `_calib_IMUtoCAM`：每个相机的 IMU->相机外参，`PoseJPL`
- `_cam_intrinsics`：每个相机的内参，`Vec`
- `_cam_intrinsics_cameras`：各相机对应的相机模型对象
- `_calib_imu_dw`：IMU 传感器标定矩阵参数（gyro/acc scale+misalignment）
- `_calib_imu_da`：IMU 传感器标定矩阵参数（gyro/acc scale+misalignment）
- `_calib_imu_tg`：gyro 重力灵敏度矩阵参数
- `_calib_imu_GYROtoIMU`：旋转从 gyro 到 IMU 参考系（Kalibr 模型）
- `_calib_imu_ACCtoIMU`：旋转从 accel 到 IMU 参考系（RPNG 模型）
- `_Cov`：当前状态协方差矩阵
- `_variables`：所有状态子变量的向量

---

## 2. `State` 构造函数里实际设置了哪些参数

在 State.cpp 中：

### 2.1 初始化活动 IMU 状态
- 创建 `_imu = std::make_shared<IMU>()`
- 设定其在 covariance 中的 `local_id`
- 将 `_imu` 加入 `_variables`

### 2.2 初始化 IMU 校准参数
- `_calib_imu_dw`：长度 6
- `_calib_imu_da`：长度 6
- 如果 `options.do_calib_imu_intrinsics` 为真，则将它们加入状态
- `_calib_imu_tg`：长度 9
- 如果 `options.do_calib_imu_g_sensitivity` 为真，则加入状态

### 2.3 初始化 IMU 参考系旋转
- `_calib_imu_GYROtoIMU`：`JPLQuat`
- `_calib_imu_ACCtoIMU`：`JPLQuat`
- 如果 `options.imu_model==KALIBR`，则加入 `_calib_imu_GYROtoIMU`
- 如果 `options.imu_model==RPNG`，则加入 `_calib_imu_ACCtoIMU`

### 2.4 初始化相机相关参数
- `_calib_dt_CAMtoIMU = std::make_shared<Vec>(1)`
- `_calib_IMUtoCAM`：每个相机一个 `PoseJPL`
- `_cam_intrinsics`：每个相机一个 `Vec(8)`
- 如果 `options.do_calib_camera_pose`，则把每个 camera pose 加入状态
- 如果 `options.do_calib_camera_intrinsics`，则把每个 camera intrinsics 加入状态

### 2.5 初始化协方差
- 默认协方差：`_Cov = (1e-3)^2 * I`
- 对校准参数设置先验：
  - `_calib_imu_dw`：`0.005^2`
  - `_calib_imu_da`：`0.008^2`
  - `_calib_imu_tg`：`0.005^2`（如果启用）
  - `_calib_imu_GYROtoIMU` / `_calib_imu_ACCtoIMU`：`0.005^2`
  - `_calib_dt_CAMtoIMU`：`0.01^2`
  - 相机外参姿态：`0.005^2`，平移：`0.015^2`
  - 相机内参：径向/主点 `1.0^2`，其余 `0.005^2`

---

## 3. `StateOptions` 里控制的参数

StateOptions.h 中的主要选项：

- `do_fej`：是否使用 First-Estimate Jacobian
- `integration_method`：IMU 推进方式 `DISCRETE/RK4/ANALYTICAL`
- `do_calib_camera_pose`：是否在线标定相机外参
- `do_calib_camera_intrinsics`：是否在线标定相机内参
- `do_calib_camera_timeoffset`：是否在线标定相机-IMU 时间偏移
- `do_calib_imu_intrinsics`：是否在线标定 IMU 内参
- `do_calib_imu_g_sensitivity`：是否在线标定 IMU 重力灵敏度
- `imu_model`：`KALIBR` 或 `RPNG`
- `max_clone_size`：滑动窗口最大 clone 数
- `max_slam_features`：最大 SLAM 特征数
- `max_slam_in_update`：单次 SLAM 更新最大特征数
- `max_msckf_in_update`：单次 MSCKF 更新最大特征数
- `max_aruco_features`：最大 ARUCO 特征数
- `num_cameras`：相机数量
- `feat_rep_msckf / feat_rep_slam / feat_rep_aruco`：特征表示形式

---

## 4. `IMU` 状态结构和更新方式

### 4.1 `IMU` 的状态组成
在 IMU.h 中：

- `_pose`：`PoseJPL`，包含：
  - 四元数 `q_JPL` (`[x,y,z,w]`)
  - 位置 `p`
- `_v`：速度
- `_bg`：陀螺偏置
- `_ba`：加速度计偏置

`IMU` 的完整状态向量长度是 16，但 `Type(15)` 表示误差状态只有 15 维（因为 quaternion 误差用 3 维表示）。

### 4.2 `IMU::update()` 的更新公式
`IMU::update(const Eigen::VectorXd &dx)` 里，`dx` 的含义为：

- `dx[0:2]`：姿态误差
- `dx[3:5]`：位置增量
- `dx[6:8]`：速度增量
- `dx[9:11]`：陀螺偏置增量
- `dx[12:14]`：加速度偏置增量

具体执行：

- `dq = normalize([0.5*dx[0:3]; 1.0])`
- `q_new = dq ⊗ q_old`
- `p_new = p_old + dx[3:6]`
- `v_new = v_old + dx[6:9]`
- `bg_new = bg_old + dx[9:12]`
- `ba_new = ba_old + dx[12:15]`

---

## 5. 姿态更新的关键点

### 5.1 `PoseJPL` 的更新方式
PoseJPL.h 中：

- 先构造扰动四元数：
  - `dq << 0.5 * dx[0:3], 1.0;`
  - `dq = quatnorm(dq)`
- 然后左乘当前 quaternion：
  - `q_new = quat_multiply(dq, quat())`

这说明：
- 使用的是 JPL 四元数
- 采用“左乘误差”模型
- `dx[0:3]` 是小角度增量

### 5.2 JPL 四元数相关规则
在 quat_ops.h 中：

- 四元数表示为 `[q1,q2,q3,q4]^T`，其中 `q4` 是 scalar
- `quat_multiply(q,p)` 实现 JPL 左乘
- `quatnorm(q)` 强制 `q4>0` 并归一化
- `quat_2_Rot(q)` 将 JPL 四元数转换为旋转矩阵

---

## 6. 位置更新的相关内容

### 6.1 `PoseJPL` 位置更新
`PoseJPL::update(const Eigen::VectorXd &dx)` 中：
- 位置增量直接累加：
  - `p_new = p_old + dx[3:6]`

这意味着 PoseJPL 的 6 维误差状态是：
- 3 维姿态误差
- 3 维位置误差

### 6.2 `IMU` 位置更新
`IMU::update` 里同样把位置增量放在 `dx[3:6]`：

- `newX.block(4,0,3,1) += dx.block(3,0,3,1);`

也就是说：
- IMU 姿态在前
- 位置在后
- 速度、偏置在后面

---

## 7. 总结

### State 里主要可调参数
- IMU 状态：姿态、位置、速度、陀螺偏置、加速度偏置
- IMU 标定：`dw/da`、`Tg`、`GYROtoIMU`/`ACCtoIMU`
- 相机标定：相机外参、相机内参、相机-IMU 时间偏移
- 滑动窗口 clone 数量、SLAM 特征数量、MSCKF 特征数量、相机数量
- 是否启用 FEJ、IMU 线性化方法、IMU 内标定类型

### 姿态/位置更新逻辑
- `PoseJPL` 用 JPL 四元数左乘更新
- `IMU` 用相同的 JPL 姿态更新 + 位置、速度、偏置累加
- 位置更新是直接加 `dx`，姿态更新是通过 `dq ⊗ q`



VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }
}

## 解释 `VioManager::VioManager(VioManagerOptions &params_)`

这是 `VioManager` 类的构造函数，用来创建和初始化整个视觉惯性管理器对象。

### 1. 函数签名

```cpp
VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false)
```

- `VioManagerOptions &params_`：
  - 以非 `const` 引用传入参数结构体。
  - 说明调用者传入的是一个可修改的参数对象，构造函数内部会把它复制到成员变量 `params`。

- `: thread_init_running(false), thread_init_success(false)`：
  - 这是初始化列表。
  - 它在构造函数体执行之前初始化成员变量。
  - 这里将两个 `std::atomic<bool>` 成员设置为 `false`，保证对象刚创建时线程初始化状态是“未运行、未成功”。

---

## 2. 构造函数体做了哪些工作

### 2.1 开始输出启动信息

```cpp
PRINT_DEBUG("=======================================\n");
PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
PRINT_DEBUG("=======================================\n");
```

这是一个简单的调试/启动提示。

### 2.2 复制参数并打印配置

```cpp
this->params = params_;
params.print_and_load_estimator();
params.print_and_load_noise();
params.print_and_load_state();
params.print_and_load_trackers();
```

- 将传入的 `params_` 复制到类成员 `params`。
- 依次调用参数打印/加载方法，确保：
  - 估计器设置
  - 噪声参数
  - 状态设置
  - 跟踪器设置
  都被加载并打印出来。

### 2.3 OpenCV 全局设置

```cpp
cv::setNumThreads(params.num_opencv_threads);
cv::setRNGSeed(0);
```

- 设置 OpenCV 的线程数。
- 设置随机种子为 0，保证可重复性。

### 2.4 创建状态对象

```cpp
state = std::make_shared<State>(params.state_options);
```

- 使用 `params.state_options` 创建 `State` 对象。
- `state` 是整个滤波器的核心状态结构。

---

## 3. 加载系统校准参数到 `state`

### 3.1 IMU 内参

```cpp
state->_calib_imu_dw->set_value(params.vec_dw);
state->_calib_imu_dw->set_fej(params.vec_dw);
...
state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);
```

- 将 IMU 误差模型、加速度/角速度标定、传感器坐标系变换等参数写入 `state`。

### 3.2 相机与 IMU 的时间偏移

```cpp
Eigen::VectorXd temp_camimu_dt;
temp_camimu_dt.resize(1);
temp_camimu_dt(0) = params.calib_camimu_dt;
state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);
```

- 将相机-IMU 时间偏移写入状态。

### 3.3 相机内外参

```cpp
state->_cam_intrinsics_cameras = params.camera_intrinsics;
for (int i = 0; i < state->_options.num_cameras; i++) {
  state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
  state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
  state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
  state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
}
```

- 将每个相机的内参和 IMU-to-camera 外参写入滤波器状态。

---

## 4. 记录统计信息文件

```cpp
if (params.record_timing_information) {
  ...
  of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
  of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
  ...
}
```

如果开启了 `record_timing_information`，构造函数会：

- 删除旧文件
- 创建目录
- 打开统计文件
- 写入 CSV 头部

---

## 5. 初始化跟踪器和更新器

### 5.1 特征跟踪器

```cpp
int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
if (params.use_klt) {
  trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(...));
} else {
  trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(...));
}
```

- 根据 `params.use_klt` 选择 KLT 跟踪器或描述子跟踪器。
- `init_max_features` 按相机数量平均分配。

### 5.2 ArUco 跟踪器（可选）

```cpp
if (params.use_aruco) {
  trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(...));
}
```

- 如果开启 ArUco，创建一个 ArUco 特征跟踪器。

### 5.3 状态传播器

```cpp
propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);
```

- 创建 IMU 状态传播器。

### 5.4 初始化器

```cpp
initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());
```

- 创建惯性初始化器，用于系统初始化阶段。

### 5.5 MSCKF 与 SLAM 更新器

```cpp
updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);
```

- 创建 MSCKF 更新器和 SLAM 更新器。

### 5.6 ZUPT 更新器（可选）

```cpp
if (params.try_zupt) {
  updaterZUPT = std::make_shared<UpdaterZeroVelocity>(...);
}
```

- 如果启用了零速度更新，则创建对应 updater。

---

## 6. 这个构造函数的角色

它负责：

- 初始化 VIO 管理器的所有组件
- 加载并写入参数到状态
- 创建跟踪器、传播器、更新器、初始化器
- 准备好整个系统接受 IMU/相机数据

但它并不执行滤波器的“实际初始化”或“估计更新”。真正的状态初始化和滤波运行是在后续的 `feed_measurement_imu()` / `track_image_and_update()` / `try_to_initialize()` 等函数里完成的。

---

## 7. 关键点总结

- 初始化列表只初始化 `thread_init_running`、`thread_init_success`。
- 构造函数内部做了大量参数加载与状态变量写入。
- 最终目标是把 `VioManager` 变成一个“可用的 VIO 管理器”，而不是直接完成滤波器初始化。
- `state`、`trackFEATS`、`trackARUCO`、`propagator`、`initializer`、`updaterMSCKF`、`updaterSLAM`、`updaterZUPT` 都在这里创建。

```cpp
  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }
  ```


## 这一段代码的作用

在 `VioManager::VioManager(...)` 里，这三行代码负责创建视觉特征跟踪器：

- `TrackKLT(...)`：使用 KLT 光流跟踪
- `TrackDescriptor(...)`：使用描述子匹配跟踪
- `TrackAruco(...)`：使用 ArUco 标签跟踪

它们都继承自 `ov_core::TrackBase`，并且将相机标定、特征数量、立体/单目、预处理等公共参数传给基类。

---

## 关键参数含义

### `state->_cam_intrinsics_cameras`
来自 `State` 的相机内参集合。
传递给跟踪器后，跟踪器知道每个相机的内参和图像尺寸，用于：
- 特征检测
- 立体匹配
- 图像坐标关联

### `init_max_features`
由这行计算得到：

```cpp
int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
```

它表示“每个相机在初始化阶段希望维护的最大特征数”。

### `state->_options.max_aruco_features`
这是 ArUco 标签数量上限，用于避免普通特征 ID 与 ArUco 特征 ID 冲突。
在 `TrackBase` 构造函数里，`currid` 会被初始化为：

```cpp
currid = 4 * (size_t)numaruco + 1;
```

因为每个 ArUco 标签有 4 个角点，所以普通特征从这个ID之后开始分配。

### `params.use_stereo`
指示是否启用立体跟踪：
- `true`：跟踪器会同时处理左右相机，对应一对图像
- `false`：只做单目时间跟踪

### `params.histogram_method`
这是 `TrackBase::HistogramMethod` 的枚举，表示是否对图像做预处理：
- `NONE`
- `HISTOGRAM`
- `CLAHE`

---

## `TrackKLT(...)` 的意义

构造函数：

```cpp
TrackKLT(state->_cam_intrinsics_cameras,
         init_max_features,
         state->_options.max_aruco_features,
         params.use_stereo,
         params.histogram_method,
         params.fast_threshold,
         params.grid_x,
         params.grid_y,
         params.min_px_dist)
```

对应 TrackKLT.h 里：

- `numfeats`：最大跟踪特征数
- `numaruco`：最高 ArUco ID
- `stereo`：是否做立体跟踪
- `histmethod`：直方图预处理方式
- `fast_threshold`：FAST 角点检测阈值
- `grid_x/grid_y`：图像分网格检测的网格数
- `min_px_dist`：新特征之间的最小像素距离

`TrackKLT` 内部说明：
- 用 OpenCV `calcOpticalFlowPyrLK` 做特征跟踪
- 支持单目时间跟踪和立体左右匹配
- 维护图像金字塔、上一帧特征、KLT 追踪窗口等

---

## `TrackDescriptor(...)` 的意义

构造函数：

```cpp
TrackDescriptor(state->_cam_intrinsics_cameras,
                init_max_features,
                state->_options.max_aruco_features,
                params.use_stereo,
                params.histogram_method,
                params.fast_threshold,
                params.grid_x,
                params.grid_y,
                params.min_px_dist,
                params.knn_ratio)
```

对应 TrackDescriptor.h 里多了一个参数：

- `knnratio`：描述子匹配的 ratio test 阈值

这个类说明：
- 使用 ORB 描述子进行特征提取
- 使用 `BruteForce-Hamming` 作匹配
- 先做检测，再做描述子匹配、比率测试、对称性验证
- 适合特征描述子匹配而不是 KLT 光流

---

## `TrackAruco(...)` 的意义

构造函数：

```cpp
TrackAruco(state->_cam_intrinsics_cameras,
           state->_options.max_aruco_features,
           params.use_stereo,
           params.histogram_method,
           params.downsize_aruco)
```

对应 TrackAruco.h：

- `cameras`：相机内参
- `numaruco`：最大 ArUco 标签 ID
- `stereo`：是否做立体 ArUco 跟踪
- `histmethod`：图像预处理方式
- `downsize`：是否把图像缩小一倍，加快 ArUco 检测

`TrackAruco` 额外特性：
- 固定用 `cv::aruco::DICT_6X6_1000`
- 使用 OpenCV ArUco 算法检测标签
- 只检测标签角点，不像 KLT/Descriptor 那样提取一般稀疏特征
- 当 `ENABLE_ARUCO_TAGS` 未编译时会直接报错退出

---

## 这三者在 `VioManager` 里的作用

在 VioManager.cpp 中：

- 如果 `params.use_klt` 为 `true`：
  - 使用 `TrackKLT` 作为主要特征跟踪器
- 否则：
  - 使用 `TrackDescriptor` 作为主要特征跟踪器
- 如果 `params.use_aruco` 为 `true`：
  - 额外创建 `TrackAruco`，用于 ArUco 标签检测/跟踪

也就是说：

- `trackFEATS`：主稀疏特征跟踪器，KLT 或 Descriptor
- `trackARUCO`：可选的 ArUco 角点跟踪器

---

## 总结

- `TrackKLT(...)`：光流跟踪，参数主要控制 FAST 检测、网格大小、最小距离
- `TrackDescriptor(...)`：描述子匹配，依赖 ORB/Hamming，与 `knn_ratio` 相关
- `TrackAruco(...)`：Aruco 标签检测，参数控制最大标签数和是否降采样

它们最重要的公共点是：都继承自 `TrackBase`，并共享相机标定、特征数据库、立体模式、预处理方式等基础行为。

```cpp
  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

```

## 1. `Propagator::propagate_and_clone(...)` 做了什么

这个函数的目标是：把当前状态从上次相机时间 `state->_timestamp` 预测到新的相机时间 `timestamp`，然后“克隆”当前 IMU 姿态到滑动窗口里。

### 1.1 时间轴处理

它先处理相机-IMU时间偏移：

- `last_prop_time_offset`：上一次传播时使用的 `t_imu - t_cam`
- `t_off_new = state->_calib_dt_CAMtoIMU->value()(0)`：当前时刻的标定偏移

然后计算积分区间：

- `time0 = state->_timestamp + last_prop_time_offset`
- `time1 = timestamp + t_off_new`

也就是：把相机时间转换到 IMU 时间，选出这段区间内的 IMU 数据。

### 1.2 选取 IMU 段并插值

它调用：

- `Propagator::select_imu_readings(imu_data, time0, time1)`

这个函数会：
- 取出包含区间端点的 IMU 读数
- 对区间边界插值得到精确 `time0` 和 `time1`
- 输出一个至少包含两个 IMU 读数的序列

这保证积分区间精确，避免直接丢弃边界片段。

### 1.3 分段传播，累积 `Φ` 和 `Q`

它用循环对每个相邻 IMU 采样段进行传播：

```cpp
Phi_summed = F * Phi_summed;
Qd_summed = F * Qd_summed * F.transpose() + Qdi;
```

对应的数学公式是：

- 状态迁移： `Φ_{k+1} = F_k * Φ_k`
- 协方差传播： `P_{k+1} = F_k P_k F_k^T + Q_k`

其中：
- `F`：当前时段的状态转移雅可比矩阵
- `Qdi`：当前时段离散过程噪声
- `Phi_summed`：最终整个区间的综合状态迁移
- `Qd_summed`：整个区间的累计噪声

这样一来，最后只需要一次 EKF 传播调用，而不是每个段都更新整个滤波器。

### 1.4 具体的运动模型和噪声

在 `predict_and_compute(...)` 中，代码做了以下转换：

#### IMU 原始测量校正

- 加速度校正：
  - `a_hat = R_ACCtoIMU * Da * (a_meas - bias_a)`
- 角速度校正：
  - `w_hat = R_GYROtoIMU * Dw * (w_meas - bias_g - Tg * a_hat)`

这里：
- `Da, Dw, Tg` 是 IMU 内参矩阵
- `R_ACCtoIMU` / `R_GYROtoIMU` 是传感器到机体的旋转

#### 离散状态平均运动方程

对于离散积分，默认模型是：

- 方向：
  - `q_{k+1} = Exp(0.5 * Ω(w_hat) * dt) * q_k`

  代码里对应：

  ```cpp
  bigO = cos(0.5*w_norm*dt)*I + sin(0.5*w_norm*dt)/w_norm * Omega(w_hat)
  new_q = quatnorm(bigO * state->_imu->quat());
  ```

- 速度：
  - `v_{k+1} = v_k + R_{G→I}^T a_hat * dt - g * dt`

- 位置：
  - `p_{k+1} = p_k + v_k * dt + 0.5 * R_{G→I}^T a_hat * dt^2 - 0.5 * g * dt^2`

这些是经典的 IMU 运动学方程。

#### 雅可比矩阵 `F` 和 `G`

函数会计算：

- `F`：状态对状态的雅可比
- `G`：状态对噪声的雅可比

然后把连续噪声转换为离散噪声：

```cpp
Qc[0:3] = σ_w^2 / dt * I
Qc[3:6] = σ_a^2 / dt * I
Qc[6:9] = σ_wb^2 * dt * I
Qc[9:12] = σ_ab^2 * dt * I
```

再计算：

```cpp
Qd = G * Qc * G^T
```

这是“连续时间噪声 -> 离散时间过程噪声”标准公式，和 Trawny 技术报告里第 129/130 式一致。

---

## 1.5 最后一步：EKF 传播和克隆

最终，`propagate_and_clone` 调用：

```cpp
StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_summed, Qd_summed);
```

这一步会把整个滤波状态（包括 IMU、IMU 内参、偏置等）按综合迁移矩阵 `Φ_summed` 和噪声 `Qd_summed` 更新协方差。

然后：

- 更新 `state->_timestamp = timestamp`
- 更新时间偏移 `last_prop_time_offset`
- 调用 `StateHelper::augment_clone(state, last_w)`

这里的“克隆”是 MSCKF/滑动窗口算法里常见的“随机克隆”：
- 在当前时间点把 IMU 姿态复制到一个新的“clone”窗口变量
- 并将其加入状态，以便后面使用多个历史 IMU 姿态进行特征三角化和观测更新

---

## 2. `InertialInitializer::initialize(...)` 做了什么

这个函数负责决定“是否以及如何初始化 VIO 系统”，它的核心工作不是直接求解状态，而是根据当前视觉与 IMU 数据选择初始化模式，并把数据截断到初始化窗口。

### 2.1 确定初始化时间窗口

它首先找出视觉特征数据库中最新的摄像头时间：

```cpp
newest_cam_time = max(feature timestamps)
```

然后设定初始化窗口开始时间：

```cpp
oldest_time = newest_cam_time - params.init_window_time - 0.10
```

也就是：用最近 `init_window_time` 秒内的视觉数据作为初始化数据。

### 2.2 清理过旧数据

- `FeatureDatabase::cleanup_measurements(oldest_time)`：删掉旧视觉观测
- IMU 数据也删掉比 `oldest_time + calib_camimu_dt` 更早的部分

这保证初始化只用“窗口内的有效数据”。

### 2.3 计算视差，判断静止或抖动

如果 `params.init_max_disparity > 0`，它会计算两个阶段的视觉视差：

- `avg_disp0`：旧一半时间窗的特征视差
- `avg_disp1`：新一半时间窗的特征视差

视差计算公式来自 `FeatureHelper::compute_disparity(...)`：

```cpp
disp = || uv1 - uv0 ||
```

然后计算平均值和标准差。

根据结果判断：

- `has_jerk = (!moving_old && moving_new)`
- `is_still = (!moving_old && !moving_new)`

其中：
- `moving_old` = `avg_disp0 > init_max_disparity`
- `moving_new` = `avg_disp1 > init_max_disparity`

也就是：
- 若前半段静止、后半段移动：判断为“抖动/拿起”
- 若两段都静止：判断为“静止”

### 2.4 选择初始化方式

```cpp
if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && params.init_imu_thresh > 0.0) {
    return init_static->initialize(...);
} else if (params.init_dyn_use && !is_still) {
    return init_dynamic->initialize(...);
}
```

简化逻辑：

- 如果满足静态初始化条件：
  - 使用 `StaticInitializer`
- 否则如果允许动态初始化并且不是静止：
  - 使用 `DynamicInitializer`
- 否则返回 `false`

### 2.5 这里的计算本质

`initialize(...)` 本身没有直接解算完整状态，它做的是：

- 选取初始化时间段
- 清理旧观测
- 基于特征视差判断运动模式
- 选择静态或动态初始化器

所以它的“计算”主要在于：
- 时间窗口截断
- 视差 `||uv1 - uv0||` 的统计
- 阈值比较判断

真正的状态解算由 `StaticInitializer` / `DynamicInitializer` 完成。

---

## 3. 两者的关系

- `Propagator::propagate_and_clone`：在系统已经初始化后，用 IMU 数据把当前滤波状态预测到下一个相机时间，并更新协方差，最后克隆当前状态。
- `InertialInitializer::initialize`：在系统尚未初始化时，判断是否可以完成初始化，并选择静态/动态初始化方法。

简而言之：

- `initialize(...)` 负责“是否可以开始 VIO”
- `propagate_and_clone(...)` 负责“初始化后如何继续运动预测”

如果你还要，我可以继续补 `StaticInitializer` 和 `DynamicInitializer` 中具体的状态求解公式。

```cpp
 // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }
  ```

  先定位 `UpdaterZeroVelocity` 的声明和实现，然后结合调用点说明其工作流程。

Searched for files matching `**/UpdaterZeroVelocity*`, 3 matches

Read [](file:///home/evk/openvins_ws/src/open_vins/ov_msckf/src/update/UpdaterZeroVelocity.h)

Read [](file:///home/evk/openvins_ws/src/open_vins/ov_msckf/src/update/UpdaterZeroVelocity.cpp#1-1), lines 1 to 260

Read [](file:///home/evk/openvins_ws/src/open_vins/ov_msckf/src/update/UpdaterZeroVelocity.cpp#260-260), lines 260 to 330

## `UpdaterZeroVelocity` 在 `VioManager` 里的作用

这行代码创建了零速度更新器：

```cpp
updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                    propagator, params.gravity_mag, params.zupt_max_velocity,
                                                    params.zupt_noise_multiplier, params.zupt_max_disparity);
```

它的目的是：当系统检测到当前时段“静止”时，不做普通视觉特征更新，而用零速度假设做一次 IMU 约束更新，避免静止时 VIO 退化。

---

## 1. 构造函数做了什么

在 UpdaterZeroVelocity.cpp 的构造函数里：

```cpp
UpdaterZeroVelocity::UpdaterZeroVelocity(...)
    : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
      _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity) {

  _gravity << 0.0, 0.0, gravity_mag;

  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // build chi2 table for 95% confidence
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}
```

它的工作是：

- 保存传入参数
- 设置重力向量 `_gravity`
- 预计算噪声方差 `_noises.sigma_*_2`
- 预建 `chi2` 阈值表，用于后面的统计检验

---

## 2. `try_update()` 的总体流程

该函数的核心是“判断是否处于静止并做零速度更新”。

调用顺序在 `VioManager::track_image_and_update()` 和 `feed_measurement_simulation()` 中：

```cpp
if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) return;
}
```

如果 `try_update` 返回 `true`，则直接跳过正常特征更新，短路为 ZUPT 更新。

---

## 3. `try_update()` 的关键计算步骤

### 3.1 选取 IMU 数据区间

计算时间区间：

- `time0 = state->_timestamp + last_prop_time_offset`
- `time1 = timestamp + t_off_new`

然后调用：

- `Propagator::select_imu_readings(imu_data, time0, time1)`

得到 `imu_recent`，保证区间端点插值准确。

---

### 3.2 对每个 IMU 采样构建残差和雅可比

状态顺序：

- `state->_imu->q()` 方向
- `state->_imu->bg()` 陀螺偏置
- `state->_imu->ba()` 加速度偏置

对于每一个相邻 IMU 读数 `i`，计算：

- `a_hat = R_ACCtoIMU * Da * (a_meas - ba)`
- `w_hat = R_GYROtoIMU * Dw * (w_meas - bg - Tg * a_hat)`

其中：

- `Da, Dw, Tg`：IMU 内参矩阵
- `R_ACCtoIMU`, `R_GYROtoIMU`：IMU 传感器坐标变换

测量残差为“真实值应为零”：

- `r_ω = -√dt / σ_w * w_hat`
- `r_a = -√dt / σ_a * (a_hat - R * gravity)`

这里 `R` 是当前 IMU 姿态矩阵，`gravity` 是 `_gravity`。

如果启用了 `integrated_accel_constraint`，则 `r_a` 会改成速度约束而非瞬时加速度约束。

也就是说，本质上用的是以下零速度假设：

- 真实角速度 `ω_true ≈ 0`
- 真实线加速度 `a_true ≈ 0`（或静止时加速度仅为重力）

---

### 3.3 构建观测雅可比 `H`

`H` 的结构也来自这段代码：

```cpp
H.block(6*i + 0, 3, 3, 3) = -w_omega * I;
H.block(6*i + 3, 0, 3, 3) = -w_accel * skew_x(R * gravity);
H.block(6*i + 3, 6, 3, 3) = -w_accel * I;
```

对应关系：

- 角速度残差对 `bg` 的雅可比是 `-I`
- 加速度残差对 `q` 的雅可比是 `-skew(R*g)`
- 加速度残差对 `ba` 的雅可比是 `-I`

如果用速度约束，则还会对 `v` 产生雅可比。

---

### 3.4 白化和噪声

测量噪声矩阵：

- `R = _zupt_noise_multiplier * I`

其中 `_zupt_noise_multiplier` 由 `params.zupt_noise_multiplier` 控制，表示“对零速度观测噪声放大”。

偏置噪声随时间累计：

```cpp
Q_bias[0:3,0:3] = dt_summed * σ_wb^2 * I
Q_bias[3:3,3:3] = dt_summed * σ_ab^2 * I
```

这表示陀螺偏置、加速度偏置随着时间的随机游走增长。

---

### 3.5 统计检验：卡方检验

计算先验协方差：

```cpp
Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
```

如果考虑偏置演化，则：

```cpp
P_marg.block(3, 3, 6, 6) += Q_bias;
```

然后得到创新协方差：

```cpp
S = H * P_marg * H.transpose() + R;
```

卡方统计量：

```cpp
chi2 = res^T * S^{-1} * res
```

这里的阈值选自 95% 分位：

- `chi2_check = chi_squared_table[res.rows()]`

接受条件：

- `chi2 <= _options.chi2_multipler * chi2_check`
- `state->_imu->vel().norm() <= _zupt_max_velocity`

---

### 3.6 额外的视差检查

如果开启 `override_with_disparity_check`，还会计算视觉特征视差：

```cpp
FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, disp_avg, disp_var, num_features);
```

判定：

- `(disp_avg < _zupt_max_disparity && num_features > 20)`

如果视差足够小，说明视觉观测也支持“没有运动”，则更容易接受 ZUPT。

---

## 4. 如果通过检测，如何更新

默认 `explicitly_enforce_zero_motion = false`，则执行：

1. 偏置协方差传播：
   - `StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);`

2. EKF 更新：
   - `StateHelper::EKFUpdate(state, Hx_order, H, res, R);`

3. 更新时间：
   - `state->_timestamp = timestamp;`

也就是说，这里不是直接把速度设为 0，而是以“零运动测量残差”更新状态，对状态和偏置进行纠正。

---

## 5. 如果显式强制零运动（当前默认不启用）

`explicitly_enforce_zero_motion = true` 时，会做另一类更新：

- 先调用 `propagate_and_clone(state, time1_cam)`
- 构造显式残差：
  - 旋转量 `log(R0 * R1^T)`
  - 位置差 `p1 - p0`
  - 速度 `v`
- 进行 EKF 更新
- 丢弃当前 clone

这会直接将后验状态强制靠近平移/旋转不动。

---

## 6. 这个更新器与 `VioManager` 的配合

在 `VioManager::feed_measurement_imu()` 中：

- `updaterZUPT->feed_imu(message, oldest_time);`

它会保存最近的 IMU 数据并清理过旧部分。

在 `track_image_and_update()` 中：

- 首先进行特征跟踪
- 然后如果满足条件，调用 `updaterZUPT->try_update(state, message.timestamp)`
- 如果返回 `true`，则直接 `return`，跳过后续的普通 MSCKF / SLAM 更新

也就是说 `UpdaterZeroVelocity` 其实是“静止时的替代更新逻辑”。

---

## 7. 公式总结

主要公式：

- 零速度残差：
  - `r_ω = -√dt / σ_w * w_hat`
  - `r_a = -√dt / σ_a * (a_hat - R*g)`
- IMU 观测校正：
  - `a_hat = R_ACCtoIMU * Da * (a_meas - ba)`
  - `w_hat = R_GYROtoIMU * Dw * (w_meas - bg - Tg * a_hat)`
- 卡方统计量：
  - `χ^2 = r^T S^{-1} r`
- 判定门限：
  - `χ^2 <= χ^2_{0.95, dof} * chi2_multiplier`
- 速度门限：
  - `||v|| <= zupt_max_velocity`

---

## 8. 你需要记住的核心点

- `UpdaterZeroVelocity` 不同于普通视觉更新，它是“零速度约束”的检测与 EKF 更新器
- 构造函数只做参数初始化和卡方阈值准备
- `try_update()` 的核心是：
  - 抽取 IMU 区间
  - 构造零运动残差
  - 用 `P`、`H`、`R` 做卡方检验
  - 通过则做一次 EKF 更新
- 它的存在可以在车辆停稳时减少视觉更新带来的退化，特别适合“轮式平台停车 / 慢速”场景


```cpp
ROS1Visualizer::ROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim)
    : _nh(nh), _app(app), _sim(sim), thread_update_running(false) {

  // Setup our transform broadcaster
  mTfBr = std::make_shared<tf::TransformBroadcaster>();

  // Create image transport
  image_transport::ImageTransport it(*_nh);

  // Setup pose and path publisher
  pub_poseimu = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("poseimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_poseimu.getTopic().c_str());
  pub_odomimu = nh->advertise<nav_msgs::Odometry>("odomimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_odomimu.getTopic().c_str());
  pub_pathimu = nh->advertise<nav_msgs::Path>("pathimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathimu.getTopic().c_str());

  // 3D points publishing
  pub_points_msckf = nh->advertise<sensor_msgs::PointCloud2>("points_msckf", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf.getTopic().c_str());
  pub_points_slam = nh->advertise<sensor_msgs::PointCloud2>("points_slam", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf.getTopic().c_str());
  pub_points_aruco = nh->advertise<sensor_msgs::PointCloud2>("points_aruco", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_aruco.getTopic().c_str());
  pub_points_sim = nh->advertise<sensor_msgs::PointCloud2>("points_sim", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_sim.getTopic().c_str());

  // Our tracking image
  it_pub_tracks = it.advertise("trackhist", 2);
  PRINT_DEBUG("Publishing: %s\n", it_pub_tracks.getTopic().c_str());

  // Groundtruth publishers
  pub_posegt = nh->advertise<geometry_msgs::PoseStamped>("posegt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_posegt.getTopic().c_str());
  pub_pathgt = nh->advertise<nav_msgs::Path>("pathgt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathgt.getTopic().c_str());

  // Loop closure publishers
  pub_loop_pose = nh->advertise<nav_msgs::Odometry>("loop_pose", 2);
  pub_loop_point = nh->advertise<sensor_msgs::PointCloud>("loop_feats", 2);
  pub_loop_extrinsic = nh->advertise<nav_msgs::Odometry>("loop_extrinsic", 2);
  pub_loop_intrinsics = nh->advertise<sensor_msgs::CameraInfo>("loop_intrinsics", 2);
  it_pub_loop_img_depth = it.advertise("loop_depth", 2);
  it_pub_loop_img_depth_color = it.advertise("loop_depth_colored", 2);

  // option to enable publishing of global to IMU transformation
  nh->param<bool>("publish_global_to_imu_tf", publish_global2imu_tf, true);
  nh->param<bool>("publish_calibration_tf", publish_calibration_tf, true);

  // Load groundtruth if we have it and are not doing simulation
  // NOTE: needs to be a csv ASL format file
  if (nh->hasParam("path_gt") && _sim == nullptr) {
    std::string path_to_gt;
    nh->param<std::string>("path_gt", path_to_gt, "");
    if (!path_to_gt.empty()) {
      DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // Load if we should save the total state to file
  // If so, then open the file and create folders as needed
  nh->param<bool>("save_total_state", save_total_state, false);
  if (save_total_state) {

    // files we will open
    std::string filepath_est, filepath_std, filepath_gt;
    nh->param<std::string>("filepath_est", filepath_est, "state_estimate.txt");
    nh->param<std::string>("filepath_std", filepath_std, "state_deviation.txt");
    nh->param<std::string>("filepath_gt", filepath_gt, "state_groundtruth.txt");

    // If it exists, then delete it
    if (boost::filesystem::exists(filepath_est))
      boost::filesystem::remove(filepath_est);
    if (boost::filesystem::exists(filepath_std))
      boost::filesystem::remove(filepath_std);

    // Create folder path to this location if not exists
    boost::filesystem::create_directories(boost::filesystem::path(filepath_est.c_str()).parent_path());
    boost::filesystem::create_directories(boost::filesystem::path(filepath_std.c_str()).parent_path());

    // Open the files
    of_state_est.open(filepath_est.c_str());
    of_state_std.open(filepath_std.c_str());
    of_state_est << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;
    of_state_std << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;

    // Groundtruth if we are simulating
    if (_sim != nullptr) {
      if (boost::filesystem::exists(filepath_gt))
        boost::filesystem::remove(filepath_gt);
      boost::filesystem::create_directories(boost::filesystem::path(filepath_gt.c_str()).parent_path());
      of_state_gt.open(filepath_gt.c_str());
      of_state_gt << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                  << std::endl;
    }
  }

  // Start thread for the image publishing
  if (_app->get_params().use_multi_threading_pubs) {
    std::thread thread([&] {
      ros::Rate loop_rate(20);
      while (ros::ok()) {
        publish_images();
        loop_rate.sleep();
      }
    });
    thread.detach();
  }
}
```


## `ROS1Visualizer::ROS1Visualizer` 构造函数解释

这个构造函数是 OpenVINS 中 ROS1 可视化器的初始化函数，它接收三个参数并进行全面的 ROS 发布器设置。让我详细解释它做了什么：

### 1. 参数说明
```cpp
ROS1Visualizer::ROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, 
                               std::shared_ptr<VioManager> app, 
                               std::shared_ptr<Simulator> sim)
```

- `nh`: ROS 节点句柄，用于创建发布器和订阅器
- `app`: VIO 管理器核心对象，包含状态估计器
- `sim`: 仿真器对象（可为空），用于仿真模式下的真值数据

### 2. 成员变量初始化
```cpp
: _nh(nh), _app(app), _sim(sim), thread_update_running(false)
```

- 保存传入的 ROS 节点句柄、VIO 应用和仿真器引用
- 初始化线程运行标志为 `false`

### 3. TF 广播器设置
```cpp
mTfBr = std::make_shared<tf::TransformBroadcaster>();
```

创建 TF 变换广播器，用于发布坐标系变换。

### 4. 图像传输设置
```cpp
image_transport::ImageTransport it(*_nh);
```

创建图像传输对象，用于发布压缩图像。

### 5. 发布器广告（Advertise Publishers）

创建多个 ROS 主题发布器：

**状态相关：**
- `poseimu`: IMU 姿态和协方差
- `odomimu`: IMU 里程计
- `pathimu`: IMU 轨迹路径

**特征点云：**
- `points_msckf`: MSCKF 特征点
- `points_slam`: SLAM 特征点  
- `points_aruco`: ArUco 标记点
- `points_sim`: 仿真特征点

**图像：**
- `trackhist`: 跟踪历史图像

**真值（Groundtruth）：**
- `posegt`: 真值姿态
- `pathgt`: 真值轨迹

**回环检测：**
- `loop_pose`: 回环姿态
- `loop_feats`: 回环特征
- `loop_extrinsic`: 回环外参
- `loop_intrinsics`: 回环内参
- `loop_depth`: 回环深度图
- `loop_depth_colored`: 彩色深度图

### 6. ROS 参数读取
```cpp
nh->param<bool>("publish_global_to_imu_tf", publish_global2imu_tf, true);
nh->param<bool>("publish_calibration_tf", publish_calibration_tf, true);
```

从 ROS 参数服务器读取配置：
- 是否发布全局到 IMU 的 TF 变换
- 是否发布标定 TF

### 7. 真值数据加载
```cpp
if (nh->hasParam("path_gt") && _sim == nullptr) {
  // 加载 ASL 格式的 CSV 真值文件
}
```

如果不是仿真模式且提供了真值文件路径，则加载真值轨迹数据。

### 8. 状态保存设置
```cpp
nh->param<bool>("save_total_state", save_total_state, false);
if (save_total_state) {
  // 创建文件，打开输出流，准备保存状态估计、标准差和真值
}
```

如果启用状态保存：
- 创建必要的目录
- 打开状态估计文件、标准差文件
- 如果是仿真模式，还打开真值文件
- 文件格式包含时间戳、四元数、位置、速度、偏置等完整状态

### 9. 多线程图像发布
```cpp
if (_app->get_params().use_multi_threading_pubs) {
  std::thread thread([&] {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
      publish_images();
      loop_rate.sleep();
    }
  });
  thread.detach();
}
```

如果启用多线程发布，则启动一个后台线程以 20Hz 频率持续发布跟踪图像。

### 总结

这个构造函数的**核心作用**是：
1. **初始化可视化组件**：保存核心对象引用
2. **设置 ROS 通信**：广告所有需要的发布器主题
3. **配置参数**：从 ROS 参数服务器读取配置
4. **准备数据源**：加载真值数据和设置文件输出
5. **启动后台服务**：如果需要，启动图像发布线程

它为整个 VIO 系统的可视化和数据发布做好了准备，使得系统能够实时发布姿态、路径、特征点云、图像等信息到 ROS 生态系统中。