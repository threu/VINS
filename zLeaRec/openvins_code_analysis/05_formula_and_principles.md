# OpenVINS 全流程公式计算与计算原理

> 本文档结合 OpenVINS 论文（Geneva et al., "OpenVINS: An Open Platform for Visual-Inertial Research", ICRA 2020 及相关技术报告）和源码，系统梳理 MSCKF 视觉惯性里程计的全流程数学模型。

---

## 1. 坐标系与状态表示

### 1.1 坐标系定义

| 符号 | 含义 |
|------|------|
| `{G}` | 全局（世界）坐标系 |
| `{I}` | IMU 体坐标系 |
| `{C_i}` | 第 i 个相机坐标系 |

### 1.2 四元数约定

OpenVINS 使用 **JPL (Jet Propulsion Laboratory)** 四元数约定：

```
q = [q_x, q_y, q_z, q_w]^T

旋转关系:
  ^G v = R(q_{GtoI}) · ^I v

四元数乘法 (对应旋转矩阵的复合):
  R(q ⊗ p) = R(q) · R(p)

局部扰动 (误差状态):
  q = δq ⊗ q̂
  δq ≈ [½δθ, 1]^T
  
  R(δq) ≈ I + ⌊δθ×⌋  (小角度近似)
```

**重要**: OpenVINS 将位姿存储为 `q_{GtoI}`（从全局到IMU的旋转），即：
- `R_GtoI = R(q_{GtoI})`：将向量从 {G} 旋转到 {I}
- 发布的位姿消息中包含 `q_{GtoI}` 和 `p_{IinG}`

### 1.3 核心状态向量

#### IMU 状态 (16维)

```
x_I = [^I_G q̄, ^G p_I, ^G v_I, b_g, b_a]^T

维度分解:
  ^I_G q̄  : JPL四元数, 4维 (G→I旋转)
  ^G p_I  : IMU 在全局系的位置, 3维
  ^G v_I  : IMU 在全局系的速度, 3维
  b_g     : 陀螺仪偏置, 3维
  b_a     : 加速度计偏置, 3维
```

#### 完整状态向量

```
x = [x_I^T, x_C^T, x_S^T, x_E^T]^T

其中:
  x_I = IMU状态 (16维)
  x_C = 滑动窗口克隆位姿: [^G p_{I_k}, ^I_k_G q̄] for k=1..m
  x_S = SLAM特征点 (3D位置 或 反向深度)
  x_E = 标定参数 (相机内外参, IMU内参, 时间偏移)
```

#### 标定状态 (x_E)

```
可选标定:
  - 相机-IMU外参: ^I q_C, ^C p_I  (每相机)
  - 相机内参: f_x, f_y, c_x, c_y, d_1..d_4  (每相机)
  - 相机-IMU时间偏移: t_d
  - IMU内参 (陀螺仪): Dw (6维: 3缩放+3轴偏差)
  - IMU内参 (加速度计): Da (6维)
  - 重力敏感性: Tg (9维)
  - IMU轴对准: q_{GYROtoIMU} 或 q_{ACCtoIMU}
```

### 1.4 误差状态

使用 JPL 四元数局部误差状态（Mourikis 2007 ICRA）：

```
x̃ = x ⊟ x̂

IMU位姿的误差:
  δθ_I   : 姿态误差 (3维), q = δq ⊗ q̂, δq ≈ [½δθ_I, 1]
  ^G p̃_I : 位置误差: ^G p_I - ^G p̂_I (3维)
  ^G ṽ_I : 速度误差: ^G v_I - ^G v̂_I (3维)
  b̃_g    : 偏置误差: b_g - b̂_g (3维)
  b̃_a    : 偏置误差: b_a - b̂_a (3维)
```

---

## 2. IMU 运动学模型

### 2.1 连续时间运动学

IMU 测量模型：

```
ω_m = ω + b_g + n_g        (陀螺仪测量)
a_m = a + b_a + n_a        (加速度计测量)

其中:
  ω    = 真实角速度 (IMU系)
  a    = 真实加速度 (IMU系, 不含重力)
  n_g  ~ N(0, σ²_g)       : 陀螺仪高斯白噪声
  n_a  ~ N(0, σ²_a)       : 加速度计高斯白噪声
  ḃ_g  ~ N(0, σ²_{bg})    : 陀螺仪偏置随机游走
  ḃ_a  ~ N(0, σ²_{ba})    : 加速度计偏置随机游走
```

IMU 运动学微分方程：

```
^I_G q̇ = ½ Ω(ω) · ^I_G q̄           (姿态)
^G ṗ_I = ^G v_I                     (位置)
^G v̇_I = R(^I_G q̄)^T · a + ^G g    (速度, 重力在全局系)
ḃ_g = n_{bg}                        (陀螺仪偏置)
ḃ_a = n_{ba}                        (加速度计偏置)

其中 Ω(ω) = [⌊ω×⌋  ω; -ω^T  0]
      ^G g = [0, 0, 9.81]^T  (全局重力)
```

### 2.2 IMU 内参校正模型

OpenVINS 支持两种 IMU 内参模型：

#### Kalibr 模型 (默认)

```
校正后的测量:
  a_corrected = Ta · (a_m - b_a)
  ω_corrected = Tg · (ω_m - b_g)

  Ta (上三角): [s_x  m_{xy}  m_{xz}]
               [0     s_y    m_{yz}]
               [0      0      s_z ]
  
  Tg (上三角): 类似结构

逆矩阵:
  Da = Ta^{-1} (上三角)
  Dw = Tg^{-1} (上三角)

原始参数化 (vec_da/vec_dw, 6维):
  kalibr: [s_x, s_y, s_z, m_{xy}, m_{xz}, m_{yz}]
  rpng:   [m_{xy}, m_{xz}, m_{yz}, s_x, s_y, s_z]
```

#### RPNG 模型

```
RPNG 模型使用下三角形式 + ACC→IMU旋转:

  a_corrected = R_{ACCtoIMU} · Da · (a_m - b_a)  (Da下三角)
  ω_corrected = R_{GYROtoIMU} · Dw · (ω_m - b_g)  (Dw下三角)
```

#### 重力敏感性 (可选)

```
ω_corrected = R_{GYROtoIMU} · Dw · (ω_m - b_g - Tg · a_corrected)

Tg (3x3矩阵, 9个参数): 建模加速度对陀螺仪测量的影响
```

### 2.3 连续时间误差状态动力学

```
δẋ = F_c · δx + G_c · n

其中 n = [n_g^T, n_a^T, n_{bg}^T, n_{ba}^T]^T  (12维噪声)

F_c (15×15 连续时间状态转移矩阵):
  [ -⌊ω̂×⌋     0     0    -I_3    0  ]  ← δθ
  [   0       0     I_3    0      0  ]  ← p̃
  [ -R^T·⌊â×⌋ 0     0     0    -R^T ]  ← ṽ
  [   0       0     0     0      0  ]  ← b̃_g
  [   0       0     0     0      0  ]  ← b̃_a

G_c (15×12 噪声雅可比):
  [ -I_3    0      0     0  ]
  [  0      0      0     0  ]
  [  0     -R^T    0     0  ]
  [  0      0      I_3   0  ]
  [  0      0      0    I_3 ]
```

---

## 3. IMU 离散传播

### 3.1 均值传播

从时间 t_k 到 t_{k+1}，Δ t = t_{k+1} - t_k:

#### 方法1: 离散积分 (DISCRETE)

```
姿态 (零阶保持):
  R_{k+1|k} = R_k · Exp(-ω̂ · Δt)
  
  ^I_G q̄_{k+1|k} = rot2quat(Exp(-ω̂ · Δt) · R(^I_G q̄_k))

速度:
  ^G v_{k+1|k} = ^G v_k + R_k^T · â · Δt - ^G g · Δt

位置:
  ^G p_{k+1|k} = ^G p_k + ^G v_k · Δt + ½R_k^T · â · Δt² - ½^G g · Δt²

偏置 (不变):
  b_{g,k+1|k} = b_{g,k}
  b_{a,k+1|k} = b_{a,k}
```

其中 `ω̂ = ω_m - b̂_g`, `â = a_m - b̂_a`（均需要经过IMU内参校正）。

#### 方法2: RK4 数值积分 (默认)

```
使用4阶Runge-Kutta积分:
  y_0 = [q_k, p_k, v_k]
  
  k1 = f(y_0, ω_k, a_k) · Δt
  k2 = f(y_0 + k1/2, (ω_k+ω_{k+1})/2, (a_k+a_{k+1})/2) · Δt
  k3 = f(y_0 + k2/2, (ω_k+ω_{k+1})/2, (a_k+a_{k+1})/2) · Δt
  k4 = f(y_0 + k3, ω_{k+1}, a_{k+1}) · Δt
  
  y_{k+1} = y_0 + (k1 + 2k2 + 2k3 + k4)/6

其中 f 使用常加速度和常角加速度模型:
  ω(t) = ω_k + (ω_{k+1}-ω_k)/Δt · (t-t_k)
  a(t) = a_k + (a_{k+1}-a_k)/Δt · (t-t_k)
```

#### 方法3: 解析积分 (ANALYTICAL)

基于闭式数值积分（假定常ω和常a），使用预先推导的解析表达式。

### 3.2 协方差传播

EKF 协方差传播方程：

```
P_{k+1|k} = Φ(t_{k+1}, t_k) · P_{k|k} · Φ(t_{k+1}, t_k)^T + Q_d

其中:
  Φ = 状态转移矩阵 (离散)
  Q_d = 离散噪声协方差
```

#### 状态转移矩阵 Φ 的计算

##### 离散方法:

```
Φ(15×15):
  Φ_{θθ} = Exp(-ω̂·Δt)
  Φ_{θ,bg} = -Exp(-ω̂·Δt) · Jr(-ω̂·Δt) · Δt
  Φ_{p,θ} = -⌊(p_{k+1}-p_k - v_k·Δt + ½g·Δt²)×⌋ · R_k^T
  Φ_{p,v} = I_3 · Δt
  Φ_{p,ba} = -½R_k^T · Δt²
  Φ_{v,θ} = -⌊(v_{k+1}-v_k + g·Δt)×⌋ · R_k^T
  Φ_{v,ba} = -R_k^T · Δt
  Φ_{θ,θ} = Exp(-ω̂·Δt)    (姿态转移)
```

##### 解析方法:

利用预积分的闭式积分 Φ_i（见 `compute_Xi_sum()` 函数）：

```
Φ_1 = ∫_0^Δt Exp(-ω̂·τ) dτ          (一阶旋转积分)
Φ_2 = ∫_0^Δt ∫_0^τ Exp(-ω̂·s) ds dτ  (二阶旋转积分)
Jr  = Jr(-ω̂·Δt)                      (SO(3)右雅可比)
Φ_3 = ∫_0^Δt Exp(-ω̂·τ)·⌊â×⌋·Jr(-ω̂·τ) dτ      (一阶混合积分)
Φ_4 = ∫_0^Δt ∫_0^τ Exp(-ω̂·s)·⌊â×⌋·Jr(-ω̂·s) ds dτ  (二阶混合积分)
```

这些积分有闭式解（基于dθ = ||ω̂||·Δt, k̂ = ω̂/||ω̂|| 以及三角函数）。

#### 离散噪声协方差 Q_d

```
Q_c (12×12 连续时间噪声密度):
  Q_c = diag(σ²_w/Δt·I₃, σ²_a/Δt·I₃, σ²_{wb}·Δt·I₃, σ²_{ab}·Δt·I₃)

Q_d = G · Q_c · G^T
    ≈ Φ · G_c · Q_c · G_c^T · Φ^T · Δt

其中 G (15×12):
  G_{θ,n_g} = -Exp(-ω̂·Δt) · Jr(-ω̂·Δt) · Δt
  G_{θ,n_a} = ...  (包含重力敏感性耦合项)
  G_{v,n_a} = -R_k^T · Δt
  G_{p,n_a} = -½R_k^T · Δt²
  G_{bg,n_bg} = Δt · I₃
  G_{ba,n_ba} = Δt · I₃
```

### 3.3 批量传播 (propagate_and_clone)

在实际代码中，传播阶段可能包含多个 IMU 测量：

```
给定 IMU 序列 {m_1, m_2, ..., m_N} 覆盖 [t_k, t_{k+1}]:

对每个子区间 i (m_i 到 m_{i+1}):
  1. 计算 F_i, Qd_i (该子区间的状态转移和噪声)
  2. 累积:
     Φ_summed = F_i · Φ_summed
     Qd_summed = F_i · Qd_summed · F_i^T + Qd_i

最终:
  P = Φ_summed · P · Φ_summed^T + Qd_summed
  x = 最后一次的子区间均值传播结果

注意: 仅IMU状态(和IMU内参)被传播。克隆位姿和SLAM特征在传播过程中保持不变。
```

### 3.4 随机克隆 (Stochastic Cloning)

传播到新的相机时间后，将当前 IMU 位姿克隆加入状态：

```
x = [x_I^T, x_C^T, ...]^T  →  x_aug = [x_I^T, x_C^T, x_{clone}^T, ...]^T

x_{clone} = [^G p_I, ^I_G q̄]^T  (在当前t_{k+1}的位姿)

协方差扩充 (线性化):
  P_aug = [ P        P · J_c^T  ]
          [ J_c·P    J_c·P·J_c^T ]

  J_c = ∂x_{clone}/∂x (雅可比)

对于带时间偏移标定的情况:
  J_c 包含 ∂x_{clone}/∂t_d 项 (使用最后角速度 ω_last)
```

---

## 4. 视觉测量模型

### 4.1 透视投影模型

3D特征点 `^G p_f` 投影到相机 `C_i`：

```
1. 从全局转换到相机坐标系:
   ^Ci p_f = R(^Ci_I q̄) · R(^I_G q̄) · (^G p_f - ^G p_I) + ^Ci p_I

   简写: ^Ci p_f = R_{CiI} · R_{IG} · (^G p_f - ^G p_I) + p_{CiI}

2. 透视投影:
   [u_n, v_n]^T = [^Ci x_f / ^Ci z_f, ^Ci y_f / ^Ci z_f]^T

3. 畸变模型 (radtan/plumb_bob):
   r² = u_n² + v_n²
   u_d = u_n·(1 + k₁r² + k₂r⁴) + 2p₁u_nv_n + p₂(r²+2u_n²)
   v_d = v_n·(1 + k₁r² + k₂r⁴) + p₁(r²+2v_n²) + 2p₂u_nv_n

4. 内参:
   u = f_x·u_d + c_x
   v = f_y·v_d + c_y

投影函数: z = h(x)
测量模型:
  z_m = h(x) + n_z
  n_z ~ N(0, R)  (R = σ_{px}² · I_2, 像素噪声)
```

### 4.2 测量雅可比 (线性化)

```
∂z/∂x = H_x = [H_θ, H_p, H_f, ...]

关键子块 (以克隆位姿为例):

H_θ: 投影对 IMU 姿态的导数
  = ∂z/∂^Ci p_f · ∂^Ci p_f/∂(^I_G q̄)

H_p: 投影对 IMU 位置的导数
  = ∂z/∂^Ci p_f · ∂^Ci p_f/∂(^G p_I)

H_f: 投影对特征点位置的导数
  = ∂z/∂^Ci p_f · ∂^Ci p_f/∂(^G p_f)
```

投影雅可比 ∂z/∂^Ci p_f:

```
∂z/∂^Ci p_f = [f_x/z_f     0      -f_x·x_f/z_f²]
              [0         f_y/z_f   -f_y·y_f/z_f²]

乘以畸变雅可比 (从畸变像素到归一化坐标)
```

---

## 5. MSCKF 更新

### 5.1 特征三角化

给定特征点在多个相机位姿中的观测，三角化其 3D 位置：

```
对于特征 f，在 m 个观测中有:
  z_j = h(x̂, ^G p_f) + n_j,  j = 1..m

线性化:
  z̃_j ≈ H_{x,j} · x̃ + H_{f,j} · ^G p̃_f + n_j

堆叠所有观测:
  z̃ = H_x · x̃ + H_f · ^G p̃_f + n

其中:
  z̃ = [z̃_1^T, ..., z̃_m^T]^T
  H_x = [H_{x,1}^T, ..., H_{x,m}^T]^T
  H_f = [H_{f,1}^T, ..., H_{f,m}^T]^T
```

求解特征位置 (最小二乘):
```
^G p̂_f = argmin ||z̃ - H_x·x̃ - H_f·^G p_f||²

假设 x̃ = 0:
  ^G p̂_f = (H_f^T · H_f)^{-1} · H_f^T · z̃
```

### 5.2 零空间投影 (MSCKF 核心)

MSCKF 的核心思想：**不将特征点加入状态向量，而是通过零空间投影消除特征位置的不确定性**。

```
H_f 的零空间: Left Nullspace(H_f)

设 V 为 H_f^T 的左零空间基:
  V^T · H_f = 0

左乘 V^T:
  V^T · z̃ = V^T · H_x · x̃ + V^T · H_f · ^G p̃_f + V^T · n
           = V^T · H_x · x̃ + 0 + n'

压缩后的残差:
  r = H₀ · x̃ + n'

其中:
  r    = V^T · z̃      (残差, (2m-3)维)
  H₀   = V^T · H_x    (压缩后的雅可比)
  n' ~ N(0, V^T·R·V)  (压缩后的噪声)
```

**物理意义**: 零空间投影将 2m 个标量测量（m个像素坐标）压缩为 (2m-3) 个有效约束（消去了特征3D位置的不确定性）。

### 5.3 多特征 MSCKF EKF 更新

MSCKF 可以一次处理多个特征（批量更新）：

```
对每个特征 i = 1..K:
  1. 三角化: ^G p̂_{f,i}
  2. 线性化: z_i = h(x̂, ^G p̂_{f,i})
  3. 计算残差: r_i = z_i - h(x̂, ^G p̂_{f,i})
  4. 零空间投影: r'_i = V_i^T · r_i, H'_i = V_i^T · H_{x,i}
  5. 卡方检验: r'_i^T · (V_i^T·R·V_i)^{-1} · r'_i < χ²_{2m-3}

堆叠所有通过检验的特征:
  r = [r'_1^T, ..., r'_K^T]^T
  H = [H'_1^T, ..., H'_K^T]^T
  n ~ N(0, blkdiag(V_1^T·R·V_1, ..., V_K^T·R·V_K))
```

### 5.4 标准 EKF 更新方程

```
1. 卡尔曼增益:
   S = H · P · H^T + R     (新息协方差)
   K = P · H^T · S^{-1}    (卡尔曼增益)

2. 状态更新:
   Δx = K · r
   x̂⁺ = x̂ ⊞ Δx

3. 协方差更新 (Joseph 形式, 更数值稳定):
   P⁺ = (I - K·H) · P · (I - K·H)^T + K · R · K^T
```

---

## 6. SLAM 特征更新

### 6.1 SLAM 特征参数化

支持多种特征表示 (`LandmarkRepresentation`):

#### GLOBAL_3D (默认, 3维)

```
x_{SLAM} = ^G p_f = [x, y, z]^T

全局坐标系中的3D位置
雅可比: 与MSCKF相同的形式
```

#### ANCHORED_MSCKF (3维)

```
x_{SLAM} = ^A p_f  (相对于锚点帧 A 的位置)

锚点帧为滑动窗口中的一个历史克隆位姿
```

#### ANCHORED_INVERSE_DEPTH (1维)

```
x_{SLAM} = 1/λ  (锚点帧中的反向深度)

^A p_f = 1/λ · [u_n, v_n, 1]^T

其中 [u_n, v_n] 是锚点帧中的归一化观测方向
```

### 6.2 SLAM 延迟初始化 (Delayed Initialization)

新的 SLAM 特征不立即加入状态，而是等待跟踪足够多帧后才初始化：

```
延迟初始化条件:
  1. 特征已被跟踪 ≥ max_clone_size 帧
  2. 特征仍在当前帧中可见
  3. 当前 SLAM 特征数 < max_slam_features

初始化步骤:
  1. 三角化: 使用所有历史观测计算 ^G p_f
  2. 增广状态: x_aug = [x; ^G p_f]
  3. 增广协方差:
     P_aug = [ P         P·J_f^T    ]
             [ J_f·P    J_f·P·J_f^T ]

     J_f = ∂^G p_f/∂x  (特征位置对状态的雅可比)
```

### 6.3 SLAM EKF 更新

```
与MSCKF不同:
  - 不进行零空间投影
  - 特征位置 ^G p_f 是状态的一部分
  - 标准EKF观测模型

测量模型:
  z = h(x_I, x_{clone}, ^G p_f) + n

线性化:
  z̃ ≈ H_x · x̃ + H_f · ^G p̃_f + n
     = [H_x, H_f] · x̃_aug + n
     = H_aug · x̃_aug + n

标准EKF更新 (与5.4节相同, 使用 H_aug)
```

### 6.4 Anchor Change

当 SLAM 特征的锚点克隆即将被边缘化时，需要切换锚点：

```
原始:  ^A p_f  (相对于锚点 A)
切换为: ^A' p_f (相对于新锚点 A')

^A' p_f = R_{A'I} · R_{IG} · (^G p_f - ^G p_I^{A'}) + p_{A'I}
        = R_{A'I} · R_{IA} · (^A p_f - ^A p_I^{A'}) + p_{A'I}

协方差传播:
  P_new = J_switch · P_old · J_switch^T
  J_switch = ∂^A' p_f / ∂[x, ^A p_f]
```

---

## 7. 边缘化 (Marginalization)

### 7.1 最老克隆的边缘化

当滑动窗口满时（默认 max_clone_size=11），边缘化最老的 IMU 位姿克隆：

```
1. 选择边缘化状态:
   x_m = 最老的 IMU 克隆 (q, p, 7维)
   (加上所有在当前边缘化时刻丢失的MSCKF特征的信息)

2. 构建先验:
   所有与 x_m 相关的测量 (视觉投影、IMU传播) 形成条件概率:
   p(x_r | x_m) ≈ N(μ + J·x_m, Σ)
   
   边际分布:
   p(x_r) = ∫ p(x_r|x_m) · p(x_m) dx_m

3. Schur Complement:
   P = [ P_{rr}  P_{rm} ]
       [ P_{mr}  P_{mm} ]
   
   P_{rr}' = P_{rr} - P_{rm} · P_{mm}^{-1} · P_{mr}
```

### 7.2 SLAM 特征的边缘化

当 SLAM 特征丢失跟踪或更新失败多次后：

```
边缘化单个 SLAM 特征 ^G p_f:
  P_new = P_rr - P_rf · P_ff^{-1} · P_fr

这消除了该特征的状态和协方差，但保留了它与剩余状态的相关性
```

### 7.3 FEJ (First Estimate Jacobians)

FEJ 确保线性化点的一致性：

```
传统 EKF:
  每次传播和更新都使用最新的状态估计作为线性化点
  可能导致估计不一致

FEJ:
  1. 状态的每个分量在首次估计时固定其线性化点
  2. 后续所有雅可比计算都使用该固定的线性化点
  3. 位姿克隆: 线性化点在克隆创建时刻固定
  4. SLAM特征: 线性化点在特征初始化时刻固定

代码实现:
  state->_imu->set_fej(value)  // 设置 FEJ 值
  state->_imu->Rot_fej()        // 获取 FEJ 旋转矩阵
```

---

## 8. 初始化

### 8.1 惯性初始化

OpenVINS 使用静止-运动惯性初始化：

```
阶段1: 静止检测
  在初始化窗口 (默认 2秒) 内:
    如果 IMU 方差小 → 检测为静止
    静止时:
      - 初始重力方向: ĝ = E[a_m] / ||E[a_m]||
      - 初始陀螺仪偏置: b̂_g = E[ω_m]
      - 初始速度: v̂ = 0

阶段2: 动态初始化 (可选)
  使用最小二乘或最大似然估计:
    min Σ ||测量 - 预测||²

  估计:
    - 初始速度 (在窗口起始时刻)
    - 重力方向细化
    - 加速度计偏置 (可观测性弱)
```

### 8.2 地面真值初始化 (仿真/serial模式)

```cpp
sys->initialize_with_gt(imustate)

imustate = [time, q_GtoI(4), p_IinG(3), v_IinG(3), bg(3), ba(3)]
```

直接使用已知状态初始化，协方差设置为较小的初始不确定性。

---

## 9. 零速更新 (ZUPT)

### 9.1 零速检测

```
检测条件:
  1. 速度估计 < zupt_max_velocity (默认 1.0 m/s)
  2. 特征视差 < zupt_max_disparity (默认 1.0 pixels)
  3. (可选) 仅初始化阶段使用

如果检测到静止:
  测量模型:
    z_v = v - 0 = v  (速度应该为零)
    
    线性化:
    ṽ = H_v · x̃ + n_v
    
    其中 n_v ~ N(0, σ_v² · I_3)
    σ_v² = zupt_noise_multiplier (默认 1.0)
```

### 9.2 零速 EKF 更新

```
标准 EKF 更新:
  r = v̂  (当前速度估计)
  H = [0_{3×3}, I_3, 0_{3×(N-6)}]  (仅作用于速度分量)
  R = σ_v² · I_3

卡尔曼更新 → 将速度压缩到零附近
```

---

## 10. 轨迹评估公式

### 10.1 ATE (绝对轨迹误差)

```
对齐后的逐点误差:
  e_{ATE,k} = ||(x_k ⊟ x̂^+_k)||₂

  位置误差: e_{pos,k} = ||p_{gt,k} - p_{est,k}^{aligned}||
  姿态误差: e_{ori,k} = ||log_so3(R_{est,k}^T · R_{gt,k})||  (度)

RMSE:
  RMSE_{pos} = √(1/K · Σ e²_{pos,k})
  RMSE_{ori} = √(1/K · Σ e²_{ori,k})
```

### 10.2 RPE (相对位姿误差)

```
对于段长 d:

相对变换:
  T_{rel,est}(k, k+d) = T_k^{-1} · T_{k+d}
  T_{rel,gt}(k, k+d)  = T_k^{-1} · T_{k+d}

误差 (在世界系):
  T_{error} = T_{rel,gt}^{-1} · T_{rel,est}
  
  e_{pos,k} = ||t_{error}||
  e_{ori,k} = ||log_so3(R_{error})||  (度)
```

### 10.3 NEES (归一化估计误差平方)

```
e_{nees,k} = (x_k ⊟ x̂_k)^T · P_k^{-1} · (x_k ⊟ x̂_k)

姿态 NEES:
  e_R = R_{gt} · R_{est}^T
  error_i = -log_so3(e_R)
  nees_{ori} = error_i^T · cov_{ori}^{-1} · error_i

位置 NEES:
  nees_{pos} = (p_{gt} - p_{est})^T · cov_{pos}^{-1} · (p_{gt} - p_{est})

一致性:
  E[NEES] = n  (n = 3, 卡方分布的自由度)
  NEES > n → 过于自信 (不一致)
  NEES < n → 过于保守
```

---

## 11. 关键参考文献

1. **Mourikis & Roumeliotis**, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation", ICRA 2007
   - MSCKF 原始论文，奠定了零空间投影方法

2. **Li & Mourikis**, "High-precision, consistent EKF-based visual-inertial odometry", IJRR 2013
   - FEJ 方法，改善 MSCKF 一致性

3. **Geneva et al.**, "OpenVINS: An Open Platform for Visual-Inertial Research", ICRA 2020
   - OpenVINS 系统论文

4. **Geneva et al.**, "OpenVINS Technical Report", 2019 (https://docs.openvins.com/)
   - 详细的数学推导和系统设计

5. **Trawny & Roumeliotis**, "Indirect Kalman Filter for 3D Attitude Estimation", UMN TR 2005
   - JPL四元数误差状态传播的基础

6. **Zhang & Scaramuzza**, "A Tutorial on Quantitative Trajectory Evaluation for Visual(-Inertial) Odometry", IROS 2018
   - ATE/RPE 评估方法

7. **Umeyama**, "Least-squares estimation of transformation parameters between two point patterns", TPAMI 1991
   - SE(3)/SIM(3) 轨迹对齐算法
