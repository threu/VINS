文件/home/evk/openvins_ws/src/open_vins/ov_msckf/src/run_simulation.cpp的解读
```cpp
/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * (版权声明与开源许可证 GPLv3 信息，此处略过翻译)
 */

// 包含处理系统信号的库，例如用于捕获终端的 Ctrl+C 中断信号
#include <csignal>
// 包含智能指针 (std::shared_ptr) 等内存管理工具
#include <memory>

// 引入 OpenVINS 核心的视觉惯性里程计 (VIO) 状态机管理器
#include "core/VioManager.h"
// 引入模拟器，用于生成虚拟的 IMU 和相机特征点数据
#include "sim/Simulator.h"
// 引入终端控制台输出颜色控制的工具（例如红色报错）
#include "utils/colors.h"
// 引入读取数据集的工具
#include "utils/dataset_reader.h"
// 引入 OpenVINS 自定义的打印日志工具
#include "utils/print.h"
// 引入传感器数据结构的定义（如 ImuData）
#include "utils/sensor_data.h"

// 预编译宏判断：如果编译时检测到系统中安装了 ROS 1
#if ROS_AVAILABLE == 1
// 引入 OpenVINS 针对 ROS 1 编写的可视化工具
#include "ros/ROS1Visualizer.h"
// 引入 ROS 1 的核心 C++ 库
#include <ros/ros.h>
// 预编译宏判断：如果编译时检测到系统中安装了 ROS 2
#elif ROS_AVAILABLE == 2
// 引入 OpenVINS 针对 ROS 2 编写的可视化工具
#include "ros/ROS2Visualizer.h"
// 引入 ROS 2 的核心 C++ 库
#include <rclcpp/rclcpp.hpp>
#endif

// 使用 ov_msckf 命名空间，这是 OpenVINS 核心算法所在的命名空间
using namespace ov_msckf;

// 声明全局的智能指针，分别用于指向：模拟器、VIO系统和可视化工具
std::shared_ptr<Simulator> sim;
std::shared_ptr<VioManager> sys;
// 根据当前使用的 ROS 版本，声明对应的可视化工具指针
#if ROS_AVAILABLE == 1
std::shared_ptr<ROS1Visualizer> viz;
#elif ROS_AVAILABLE == 2
std::shared_ptr<ROS2Visualizer> viz;
#endif

// 定义信号处理函数：当进程收到 ctrl-c (SIGINT) 信号时被调用
// 参数 signum 是接收到的信号编号，std::exit 会安全地结束程序
void signal_callback_handler(int signum) { std::exit(signum); }

// 主程序入口
int main(int argc, char **argv) {

  // 确保我们有一个配置文件路径，默认给一个提示性的假路径
  std::string config_path = "unset_path_to_config.yaml";
  // 如果用户在命令行运行此程序时传入了参数，则将第一个参数作为配置文件路径
  if (argc > 1) {
    config_path = argv[1];
  }

// ROS 节点初始化模块
#if ROS_AVAILABLE == 1
  // 启动并初始化 ROS 1 节点，节点名称为 "run_simulation"
  ros::init(argc, argv, "run_simulation");
  // 创建一个私有的 NodeHandle ("~")，用于读取该节点下的私有参数
  auto nh = std::make_shared<ros::NodeHandle>("~");
  // 尝试从 ROS 参数服务器中读取 "config_path"，如果没读到，则使用前面解析到的 config_path
  nh->param<std::string>("config_path", config_path, config_path);
#elif ROS_AVAILABLE == 2
  // 启动并初始化 ROS 2 节点
  rclcpp::init(argc, argv);
  // 配置 ROS 2 节点的选项
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true); // 允许使用未提前声明的参数
  options.automatically_declare_parameters_from_overrides(true); // 自动从外部覆盖中声明参数
  // 创建 ROS 2 节点实体
  auto node = std::make_shared<rclcpp::Node>("run_simulation", options);
  // 从 ROS 2 节点中获取 "config_path" 参数
  node->get_parameter<std::string>("config_path", config_path);
#endif

  // 创建一个 YAML 解析器对象，用于读取指定路径的配置文件
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
// 将 ROS 节点句柄传给解析器，以便在需要时解析器也能直接从 ROS 参数服务器读数据
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#elif ROS_AVAILABLE == 2
  parser->set_node(node);
#endif

  // 读取日志输出级别（Verbosity）参数，默认为 "INFO"
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  // 设置 OpenVINS 的全局打印级别（如 INFO, DEBUG, WARNING）
  ov_core::Printer::setPrintLevel(verbosity);

  // 初始化 VIO 系统所需的参数结构体
  VioManagerOptions params;
  // 通过解析器从配置文件中读取 VIO 核心参数，并在终端打印出来
  params.print_and_load(parser);
  // 读取仿真器（Simulator）特有的参数并打印
  params.print_and_load_simulation(parser);
  // 将 OpenCV 的线程数设置为 0。在仿真环境下这很重要，单线程能保证每次运行的随机性和结果是完全可重复的 (repeatability)
  params.num_opencv_threads = 0; 
  // 关闭多线程发布和订阅，同样是为了保证仿真的时序一致性和可重复性
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  
  // 使用配置好的参数实例化模拟器
  sim = std::make_shared<Simulator>(params);
  // 使用配置好的参数实例化 VIO 管理器
  sys = std::make_shared<VioManager>(params);
  
// 根据 ROS 版本实例化可视化工具，并将系统和模拟器的指针传给它，方便提取数据画图
#if ROS_AVAILABLE == 1
  viz = std::make_shared<ROS1Visualizer>(nh, sys, sim);
#elif ROS_AVAILABLE == 2
  viz = std::make_shared<ROS2Visualizer>(node, sys, sim);
#endif

  // 检查 YAML 解析器是否成功读取了所有必需的参数
  if (!parser->successful()) {
    // 如果失败，打印红色错误提示并终止程序
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //================================= 系统初始化阶段 ====================================
  //===================================================================================

  // 获取系统的初始真实状态（Ground Truth），用于完美初始化滤波器
  // 注意：我们获取的是“下一个”时间步的状态，以此来对齐我们将收到的第一个 IMU 消息的时间
  double next_imu_time = sim->current_timestamp() + 1.0 / params.sim_freq_imu;
  Eigen::Matrix<double, 17, 1> imustate; // 17维的向量代表IMU状态（四元数、位置、速度、零偏等）
  // 从模拟器中提取该时刻的真实状态
  bool success = sim->get_state(next_imu_time, imustate);
  if (!success) {
    PRINT_ERROR(RED "[SIM]: Could not initialize the filter to the first state\n" RESET);
    PRINT_ERROR(RED "[SIM]: Did the simulator load properly???\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // 模拟器返回的状态时间戳是基于“相机参考系”的
  // 因此，需要减去相机到 IMU 的时间偏移量（calib_camimu_dt），将其转换为真正的 IMU 时间戳
  imustate(0, 0) -= sim->get_true_parameters().calib_camimu_dt;

  // 使用完美的真实状态 (Ground Truth) 来初始化我们的 MSCKF 滤波器
  sys->initialize_with_gt(imustate);

  //===================================================================================
  //================================= 主数据循环阶段 ====================================
  //===================================================================================

  // 定义变量，用于缓存（Buffer）上一帧相机的数据
  // 因为视觉处理往往需要时间错位或等待下一帧，这里做了简单的缓存机制
  double buffer_timecam = -1; // 缓存的相机时间戳
  std::vector<int> buffer_camids; // 缓存的相机ID列表（多目系统）
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> buffer_feats; // 缓存的特征点数据

  // 开始循环步进处理仿真数据
#if ROS_AVAILABLE == 1
  // ROS 1 环境：只要模拟器正常且 ROS 未被关闭就一直循环
  while (sim->ok() && ros::ok()) {
#elif ROS_AVAILABLE == 2
  // ROS 2 环境同理
  while (sim->ok() && rclcpp::ok()) {
#else
  // 无 ROS 环境：绑定 Ctrl+C 信号，只要模拟器正常就一直循环
  signal(SIGINT, signal_callback_handler);
  while (sim->ok()) {
#endif

    // --- 1. 处理 IMU 数据 ---
    ov_core::ImuData message_imu;
    // 尝试从模拟器获取下一个 IMU 测量值（包含时间戳、角速度 wm、线加速度 am）
    bool hasimu = sim->get_next_imu(message_imu.timestamp, message_imu.wm, message_imu.am);
    if (hasimu) {
      // 如果获取到了，就喂给 VIO 系统的 IMU 传播环节（进行积分预测）
      sys->feed_measurement_imu(message_imu);
// 发送当前的里程计位姿供 RViz 实时显示
#if ROS_AVAILABLE == 1 || ROS_AVAILABLE == 2
      viz->visualize_odometry(message_imu.timestamp);
#endif
    }

    // --- 2. 处理 相机 数据 ---
    double time_cam;
    std::vector<int> camids;
    std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> feats;
    // 尝试从模拟器获取下一帧相机的特征点测量值
    // （模拟器直接输出特征点坐标，跳过了图像提取的过程，所以速度极快）
    bool hascam = sim->get_next_cam(time_cam, camids, feats);
    if (hascam) {
      // 这里的逻辑是：只有当缓存区有数据时（也就是从第二帧开始），
      // 才把【上一帧】缓存好的特征点喂给 VIO 系统进行更新计算
      if (buffer_timecam != -1) {
        sys->feed_measurement_simulation(buffer_timecam, buffer_camids, buffer_feats);
// 每处理完一帧相机数据，就更新一次全局可视化（如特征点轨迹、完整位姿等）
#if ROS_AVAILABLE == 1 || ROS_AVAILABLE == 2
        viz->visualize();
#endif
      }
      // 将刚刚拿到的【当前帧】数据存入缓存，供下一次循环使用
      buffer_timecam = time_cam;
      buffer_camids = camids;
      buffer_feats = feats;
    }
  }

  // 当循环结束（例如仿真数据跑完，或者用户按下了 Ctrl+C）
  // 执行最终的可视化清理和 ROS 节点关闭操作
#if ROS_AVAILABLE == 1
  viz->visualize_final();
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  viz->visualize_final();
  rclcpp::shutdown();
#endif

  // 程序正常退出
  return EXIT_SUCCESS;
}
```


`Propagator.h`
```cpp
/**
   * @brief compute the Jacobians for Dw
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Dw,kalibr} & =
   *   \begin{bmatrix}
   *   {}^w\hat{w}_1 \mathbf{I}_3  & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_2\mathbf{e}_3 & {}^w\hat{w}_3 \mathbf{e}_3
   *   \end{bmatrix} \\
   *   \mathbf{H}_{Dw,rpng} & =
   *   \begin{bmatrix}
   *   {}^w\hat{w}_1\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_3 \mathbf{I}_3
   *   \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param w_uncorrected Angular velocity in a frame with bias and gravity sensitivity removed
   */
  static Eigen::MatrixXd compute_H_Dw(std::shared_ptr<State> state, const Eigen::Vector3d &w_uncorrected);
  ```
$$\begin{align}
    \mathbf{H}_{Dw,kalibr} & =
      \begin{bmatrix}
      {}^w\hat{w}_1 \mathbf{I}_3  & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_2\mathbf{e}_3 & {}^w\hat{w}_3 \mathbf{e}_3
      \end{bmatrix} \\
      \mathbf{H}_{Dw,rpng} & =
     \begin{bmatrix}
      {}^w\hat{w}_1\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_3 \mathbf{I}_3
     \end{bmatrix}
  \end{align}
 $$

 ```cpp
 /**
   * @brief compute the Jacobians for Da
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Da,kalibr} & =
   * \begin{bmatrix}
   *   {}^a\hat{a}_1\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_3 \mathbf{I}_3
   * \end{bmatrix} \\
   * \mathbf{H}_{Da,rpng} & =
   * \begin{bmatrix}
   *   {}^a\hat{a}_1 \mathbf{I}_3 &  & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_2\mathbf{e}_3 & {}^a\hat{a}_3\mathbf{e}_3
   * \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param a_uncorrected Linear acceleration in gyro frame with bias removed
   */
  static Eigen::MatrixXd compute_H_Da(std::shared_ptr<State> state, const Eigen::Vector3d &a_uncorrected);
```
$$
    \begin{align}
    \mathbf{H}_{Da,kalibr} & =
    \begin{bmatrix}
      {}^a\hat{a}_1\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_3 \mathbf{I}_3
    \end{bmatrix} \\
    \mathbf{H}_{Da,rpng} & =
    \begin{bmatrix}
      {}^a\hat{a}_1 \mathbf{I}_3 &  & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_2\mathbf{e}_3 & {}^a\hat{a}_3\mathbf{e}_3
    \end{bmatrix}
   \end{align}
$$
```cpp
  /**
   * @brief compute the Jacobians for Tg
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Tg} & =
   *  \begin{bmatrix}
   *  {}^I\hat{a}_1 \mathbf{I}_3 & {}^I\hat{a}_2 \mathbf{I}_3 & {}^I\hat{a}_3 \mathbf{I}_3
   *  \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param a_inI Linear acceleration with bias removed
   */
  static Eigen::MatrixXd compute_H_Tg(std::shared_ptr<State> state, const Eigen::Vector3d &a_inI);
```
$$\begin{align}
\mathbf{H}_{Tg} & =
    \begin{bmatrix}
    {}^I\hat{a}_1 \mathbf{I}_3 & {}^I\hat{a}_2 \mathbf{I}_3 & {}^I\hat{a}_3 \mathbf{I}_3
   \end{bmatrix}
   \end{align}
$$
```cpp
protected:
  /**
   * @brief Propagates the state forward using the imu data and computes the noise covariance and state-transition
   * matrix of this interval.
   *
   * This function can be replaced with analytical/numerical integration or when using a different state representation.
   * This contains our state transition matrix along with how our noise evolves in time.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref propagation_discrete page for details on how discrete model was derived.
   * See the @ref propagation_analytical page for details on how analytic model was derived.
   *
   * @param state Pointer to state
   * @param data_minus imu readings at beginning of interval
   * @param data_plus imu readings at end of interval
   * @param F State-transition matrix over the interval
   * @param Qd Discrete-time noise covariance over the interval
   */
  void predict_and_compute(std::shared_ptr<State> state, const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus,
                           Eigen::MatrixXd &F, Eigen::MatrixXd &Qd);
```

```cpp

  /**
   * @brief Discrete imu mean propagation.
   *
   * See @ref disc_prop for details on these equations.
   * \f{align*}{
   * \text{}^{I_{k+1}}_{G}\hat{\bar{q}}
   * &= \exp\bigg(\frac{1}{2}\boldsymbol{\Omega}\big({\boldsymbol{\omega}}_{m,k}-\hat{\mathbf{b}}_{g,k}\big)\Delta t\bigg)
   * \text{}^{I_{k}}_{G}\hat{\bar{q}} \\
   * ^G\hat{\mathbf{v}}_{k+1} &= \text{}^G\hat{\mathbf{v}}_{I_k} - {}^G\mathbf{g}\Delta t
   * +\text{}^{I_k}_G\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t\\
   * ^G\hat{\mathbf{p}}_{I_{k+1}}
   * &= \text{}^G\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k} \Delta t
   * - \frac{1}{2}{}^G\mathbf{g}\Delta t^2
   * + \frac{1}{2} \text{}^{I_k}_{G}\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t^2
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                             Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);
```
$$\begin{align}
    \text{}^{I_{k+1}}_{G}\hat{\bar{q}}
    &= \exp\bigg(\frac{1}{2}\boldsymbol{\Omega}\big({\boldsymbol{\omega}}_{m,k}-\hat{\mathbf{b}}_{g,k}\big)\Delta t\bigg)
    \text{}^{I_{k}}_{G}\hat{\bar{q}} \\
    ^G\hat{\mathbf{v}}_{k+1} &= \text{}^G\hat{\mathbf{v}}_{I_k} - {}^G\mathbf{g}\Delta t
    +\text{}^{I_k}_G\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t\\
    ^G\hat{\mathbf{p}}_{I_{k+1}}
    &= \text{}^G\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k} \Delta t
    - \frac{1}{2}{}^G\mathbf{g}\Delta t^2
    + \frac{1}{2} \text{}^{I_k}_{G}\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t^2
   \end{align}
$$
```cpp
  /**
   * @brief RK4 imu mean propagation.
   *
   * See this wikipedia page on [Runge-Kutta Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).
   * We are doing a RK4 method, [this wolfram page](http://mathworld.wolfram.com/Runge-KuttaMethod.html) has the forth order equation
   * defined below. We define function \f$ f(t,y) \f$ where y is a function of time t, see @ref imu_kinematic for the definition of the
   * continuous-time functions.
   *
   * \f{align*}{
   * {k_1} &= f({t_0}, y_0) \Delta t  \\
   * {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
   * {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
   * {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
   * y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1 \over 3}{k_3} + {1 \over 6}{k_4}} \right)
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_rk4(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                        const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v,
                        Eigen::Vector3d &new_p);
```
$$\begin{align}
    {k_1} &= f({t_0}, y_0) \Delta t  \\
    {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
    {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
    {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
    y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1 \over 3}{k_3} + {1 \over 6}{k_4}} \right)
   \end{align}
$$
```cpp
  /**
   * @brief Analytically compute the integration components based on ACI^2
   *
   * See the @ref analytical_prop page and @ref analytical_integration_components for details.
   * For computing Xi_1, Xi_2, Xi_3 and Xi_4 we have:
   *
   * \f{align*}{
   * \boldsymbol{\Xi}_1 & = \mathbf{I}_3 \delta t + \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}} \lfloor \hat{\mathbf{k}} \rfloor
   * + \left(\delta t  - \frac{\sin (\hat{\omega} \delta t)}{\hat{\omega}}\right) \lfloor \hat{\mathbf{k}} \rfloor^2 \\
   * \boldsymbol{\Xi}_2 & = \frac{1}{2} \delta t^2 \mathbf{I}_3 +
   * \frac{\hat{\omega} \delta t - \sin (\hat{\omega} \delta t)}{\hat{\omega}^2}\lfloor \hat{\mathbf{k}} \rfloor
   * + \left( \frac{1}{2} \delta t^2 - \frac{1  - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) \lfloor \hat{\mathbf{k}} \rfloor ^2
   * \\ \boldsymbol{\Xi}_3  &= \frac{1}{2}\delta t^2  \lfloor \hat{\mathbf{a}} \rfloor
   * + \frac{\sin (\hat{\omega} \delta t_i) - \hat{\omega} \delta t }{\hat{\omega}^2} \lfloor\hat{\mathbf{a}} \rfloor \lfloor
   * \hat{\mathbf{k}} \rfloor
   * + \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)  }{\hat{\omega}^2}
   * \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
   * + \left( \frac{1}{2} \delta t^2 - \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) 	\lfloor\hat{\mathbf{a}} \rfloor
   * \lfloor \hat{\mathbf{k}} \rfloor ^2
   * + \left(
   * \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
   *  \right)
   *  \lfloor \hat{\mathbf{k}} \rfloor ^2 \lfloor\hat{\mathbf{a}} \rfloor
   *  + \left(
   *  \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
   *  \right)  \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
   *  - \frac{ 3 \sin (\hat{\omega} \delta t) - 2 \hat{\omega} \delta t - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)
   * }{\hat{\omega}^2} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor ^2  \\
   * \boldsymbol{\Xi}_4 & = \frac{1}{6}\delta
   * t^3 \lfloor\hat{\mathbf{a}} \rfloor
   * + \frac{2(1 - \cos (\hat{\omega} \delta t)) - (\hat{\omega}^2 \delta t^2)}{2 \hat{\omega}^3}
   *  \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor
   *  + \left(
   *  \frac{2(1- \cos(\hat{\omega} \delta t)) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t)}{\hat{\omega}^3}
   *  \right)
   *  \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
   *  + \left(
   *  \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t}{\hat{\omega}^3} +
   *  \frac{\delta t^3}{6}
   *  \right)
   *  \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor^2
   *  +
   *  \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
   * \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \lfloor \hat{\mathbf{k}} \rfloor^2\lfloor\hat{\mathbf{a}} \rfloor
   *  +
   *  \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
   * \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
   *  +
   *  \frac{4 \cos(\hat{\omega} \delta t) - 4 + (\hat{\omega} \delta t)^2 + \hat{\omega} \delta t \sin(\hat{\omega} \delta t) }
   *  {\hat{\omega}^3}
   *  \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor^2
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4 in order
   */
  void compute_Xi_sum(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                      Eigen::Matrix<double, 3, 18> &Xi_sum);
```
$$\begin{align}
    \boldsymbol{\Xi}_1 & = \mathbf{I}_3 \delta t + \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}} \lfloor \hat{\mathbf{k}} \rfloor
    + \left(\delta t  - \frac{\sin (\hat{\omega} \delta t)}{\hat{\omega}}\right) \lfloor \hat{\mathbf{k}} \rfloor^2 \\
    \boldsymbol{\Xi}_2 & = \frac{1}{2} \delta t^2 \mathbf{I}_3 +
    \frac{\hat{\omega} \delta t - \sin (\hat{\omega} \delta t)}{\hat{\omega}^2}\lfloor \hat{\mathbf{k}} \rfloor
    + \left( \frac{1}{2} \delta t^2 - \frac{1  - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) \lfloor \hat{\mathbf{k}} \rfloor ^2
    \\ \boldsymbol{\Xi}_3  &= \frac{1}{2}\delta t^2  \lfloor \hat{\mathbf{a}} \rfloor
    + \frac{\sin (\hat{\omega} \delta t_i) - \hat{\omega} \delta t }{\hat{\omega}^2} \lfloor\hat{\mathbf{a}} \rfloor \lfloor
    \hat{\mathbf{k}} \rfloor
    + \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)  }{\hat{\omega}^2}
    \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
    + \left( \frac{1}{2} \delta t^2 - \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) 	\lfloor\hat{\mathbf{a}} \rfloor
    \lfloor \hat{\mathbf{k}} \rfloor ^2
    + \left(
    \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
     \right)
     \lfloor \hat{\mathbf{k}} \rfloor ^2 \lfloor\hat{\mathbf{a}} \rfloor
     + \left(
     \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
     \right)  \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
     - \frac{ 3 \sin (\hat{\omega} \delta t) - 2 \hat{\omega} \delta t - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)
    }{\hat{\omega}^2} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor ^2  \\
    \boldsymbol{\Xi}_4 & = \frac{1}{6}\delta
    t^3 \lfloor\hat{\mathbf{a}} \rfloor
    + \frac{2(1 - \cos (\hat{\omega} \delta t)) - (\hat{\omega}^2 \delta t^2)}{2 \hat{\omega}^3}
     \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor
     + \left(
     \frac{2(1- \cos(\hat{\omega} \delta t)) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t)}{\hat{\omega}^3}
     \right)
     \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
     + \left(
     \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t}{\hat{\omega}^3} +
     \frac{\delta t^3}{6}
     \right)
     \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor^2
     +
     \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
    \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \lfloor \hat{\mathbf{k}} \rfloor^2\lfloor\hat{\mathbf{a}} \rfloor
     +
     \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
    \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
     +
     \frac{4 \cos(\hat{\omega} \delta t) - 4 + (\hat{\omega} \delta t)^2 + \hat{\omega} \delta t \sin(\hat{\omega} \delta t) }
     {\hat{\omega}^3}
     \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor^2
   \end{align}
$$
```cpp
  /**
   * @brief Analytically predict IMU mean based on ACI^2
   *
   * See the @ref analytical_prop page for details.
   *
   * \f{align*}{
   * {}^{I_{k+1}}_G\hat{\mathbf{R}} & \simeq  \Delta \mathbf{R}_k {}^{I_k}_G\hat{\mathbf{R}}  \\
   * {}^G\hat{\mathbf{p}}_{I_{k+1}} & \simeq {}^{G}\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k}\delta t_k  +
   * {}^{I_k}_G\hat{\mathbf{R}}^\top  \Delta \hat{\mathbf{p}}_k - \frac{1}{2}{}^G\mathbf{g}\delta t^2_k \\
   * {}^G\hat{\mathbf{v}}_{I_{k+1}} & \simeq  {}^{G}\hat{\mathbf{v}}_{I_k} + {}^{I_k}_G\hat{\mathbf{R}}^\top + \Delta \hat{\mathbf{v}}_k -
   * {}^G\mathbf{g}\delta t_k
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4
   */
  void predict_mean_analytic(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                             Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p, Eigen::Matrix<double, 3, 18> &Xi_sum);
```
$$\begin{align}
    {}^{I_{k+1}}_G\hat{\mathbf{R}} & \simeq  \Delta \mathbf{R}_k {}^{I_k}_G\hat{\mathbf{R}}  \\
    {}^G\hat{\mathbf{p}}_{I_{k+1}} & \simeq {}^{G}\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k}\delta t_k  +
    {}^{I_k}_G\hat{\mathbf{R}}^\top  \Delta \hat{\mathbf{p}}_k - \frac{1}{2}{}^G\mathbf{g}\delta t^2_k \\
    {}^G\hat{\mathbf{v}}_{I_{k+1}} & \simeq  {}^{G}\hat{\mathbf{v}}_{I_k} + {}^{I_k}_G\hat{\mathbf{R}}^\top + \Delta \hat{\mathbf{v}}_k -
    {}^G\mathbf{g}\delta t_k
   \end{align}
$$
```cpp
  /**
   * @brief Analytically compute state transition matrix F and noise Jacobian G based on ACI^2
   *
   * This function is for analytical integration of the linearized error-state.
   * This contains our state transition matrix and noise Jacobians.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref analytical_linearization page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param w_uncorrected Angular velocity in acc frame with bias and gravity sensitivity removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4
   * @param F State transition matrix
   * @param G Noise Jacobian
   */
  void compute_F_and_G_analytic(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                                const Eigen::Vector3d &w_uncorrected, const Eigen::Vector3d &a_uncorrected, const Eigen::Vector4d &new_q,
                                const Eigen::Vector3d &new_v, const Eigen::Vector3d &new_p, const Eigen::Matrix<double, 3, 18> &Xi_sum,
                                Eigen::MatrixXd &F, Eigen::MatrixXd &G);

  /**
   * @brief compute state transition matrix F and noise Jacobian G
   *
   * This function is for analytical integration or when using a different state representation.
   * This contains our state transition matrix and noise Jacobians.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref error_prop page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param w_uncorrected Angular velocity in acc frame with bias and gravity sensitivity removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param F State transition matrix
   * @param G Noise Jacobian
   */
  void compute_F_and_G_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                                const Eigen::Vector3d &w_uncorrected, const Eigen::Vector3d &a_uncorrected, const Eigen::Vector4d &new_q,
                                const Eigen::Vector3d &new_v, const Eigen::Vector3d &new_p, Eigen::MatrixXd &F, Eigen::MatrixXd &G);

  /// Container for the noise values
  NoiseManager _noises;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;
  std::mutex imu_data_mtx;

  /// Gravity vector
  Eigen::Vector3d _gravity;

  // Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  // Cache of the last fast propagated state
  std::atomic<bool> cache_imu_valid;
  double cache_state_time;
  Eigen::MatrixXd cache_state_est;
  Eigen::MatrixXd cache_state_covariance;
  double cache_t_off;
};

} // namespace ov_msckf
```