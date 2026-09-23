# 鲁棒的实时激光雷达-惯性初始化

**Robust Real-time LiDAR-inertial Initialization**

Fangcheng Zhu\*（朱方成），Yunfan Ren\*（任云帆），Fu Zhang（张富）

> 已被 2022 年 IEEE/RSJ 智能机器人与系统国际会议（IROS，日本京都）接收

\* 这两位作者对本工作贡献相同。

F. Zhu、Y. Ren 和 F. Zhang 来自香港大学机械工程系。{zhufc,renyf}@connect.hku.hk, fuzhang@hku.hk

---

## 摘要

对于大多数激光雷达-惯性里程计而言，准确的初始状态——包括激光雷达与 6 轴 IMU 之间的时间偏移和外参变换——起着重要作用，通常被视为先决条件。然而，在自行定制的激光雷达-惯性系统中，这些信息可能并非总是可用。在本文中，我们提出了 **LI-Init**：一个完整且实时的激光雷达-惯性系统初始化过程，它通过将激光雷达测量所估计的状态与 IMU 所测量的状态对齐，来标定激光雷达与 IMU 之间的时间偏移和外参参数，以及重力向量和 IMU 偏置。我们将所提出的方法实现为一个初始化模块，该模块能够自动检测所采集数据的激励程度，并在线标定时间偏移、外参、重力向量和 IMU 偏置，随后将这些作为实时激光雷达-惯性里程计系统的高质量初始状态值。使用不同类型激光雷达和激光雷达-惯性组合进行的实验表明，我们的初始化方法具有鲁棒性、适应性和高效性。我们的激光雷达-惯性初始化过程 LI-Init 的实现和测试数据已在 Github¹ 上开源，并已集成到最先进的激光雷达-惯性里程计系统 FAST-LIO2 中。

**图 1.** 实验平台，包括多个激光雷达（非重复扫描的 Livox Mid360、机械旋转的 Hesai PandarXT、小视场的 Livox Avia）以及激光雷达内置 IMU 和 Pixhawk 飞控。RealSense L515 相机仅用于以第一人称视角录制视频。

---

## I. 引言

传感器被称为机器人的眼睛，它们赋予机器人探索周围环境以及执行自定位和导航的能力。相机是一种常用的传感器，因为它能够以低成本、轻重量提供丰富的 RGB 信息，但它容易受到光照不足的影响，且缺乏直接的深度测量，导致在重建三维环境时计算复杂度较高。与相机相比，光检测与测距（LiDAR）传感器能够提供直接、精确的三维测量，并且对光照变化具有鲁棒性，使其成为机器人定位 [1]、[2] 和建图 [3] 应用的首选。

为了应对传感器失效等突发状况并保持整个系统的鲁棒性，多传感器融合近年来成为主要趋势。惯性测量单元（IMU）是与相机或激光雷达融合的绝佳互补传感器，因为它能够在没有任何外部参考的情况下提供短期的自运动估计。IMU 是缓解由退化引起的短期里程计失效的理想选择，例如相机的昏暗光照场景以及激光雷达的无结构环境。此外，来自 IMU 的高频运动学测量有助于补偿激光雷达扫描的运动畸变，尤其是在机器人处于高速运动时 [4]。越来越多基于多传感器的同时定位与建图（SLAM）方法涌现出来，包括视觉-惯性系统 [5]、[6]、激光雷达-惯性系统 [1]、[7]、[8] 以及激光雷达-惯性-视觉系统 [9]、[10]。

由于强非线性，传感器融合系统的性能严重依赖于高效的初始化模块所提供的准确初始状态。视觉-惯性系统的初始化已被广泛研究 [11]–[13]，但很少有研究关注激光雷达-惯性系统的初始化，而后者由于以下原因是必要的：1）对于自行组装的设备，激光雷达和 IMU 通常时间不同步且外参未知，需要提前进行额外且费力的时间和空间标定。2）一次激光雷达扫描中的点是在不同时刻采样的，导致不可避免的运动畸变。在时间偏移未知的情况下，[1]、[14]、[15] 所采用的 IMU 辅助运动畸变补偿方法不再可行。3）IMU 原始测量受到显著噪声的影响，线性加速度和角速度的真实值与未知偏置耦合在一起。所有这些挑战促使我们去寻找一种完善的激光雷达-惯性初始化方法，它能够提供高质量的初始状态，包括外参变换、重力向量、IMU 偏置，并在无需任何专用硬件设置的情况下同步这两个传感器。

基于此，我们提出了一种快速、鲁棒的激光雷达-惯性初始化方法，它能够自动、准确地标定时间偏移，并在无需任何靶标或额外传感器的情况下提供可接受的初始状态，使得激光雷达-惯性里程计能够在定制的传感器设置上运行，而无需任何专用的预标定或硬件设置。我们的贡献如下：

- 我们提出了一种高效、准确、无需硬件的时间标定方法，该方法基于互相关和统一的时间-空间优化，用于估计未知但恒定的激光雷达-惯性时间偏移。
- 我们提出了一种新颖的优化公式来进行空间初始化，并提出了一种评估数据激励程度的方法。通过进一步将激光雷达估计的状态与经降噪处理的 IMU 测量对齐，我们的初始化能够自动提取初始化数据，并在运行过程中在线估计外参变换、重力向量、陀螺仪偏置和加速度计偏置。
- 我们在多种类型的激光雷达和激光雷达-惯性组合上进行了实验（见图 1），以验证我们初始化过程的高效性和准确性。据我们所知，所提出的方法是首个开源的、面向三维激光雷达-惯性系统的时间与空间初始化算法，同时支持机械旋转激光雷达和非重复扫描激光雷达。

---

## II. 相关工作

视觉-惯性系统有着各种各样的初始化方法。例如，Mur-Artal 等人 [11] 提出了一种名为 VI-ORB-SLAM 的高效 IMU 初始化方法。该方法将初始化问题划分为三个简单的子问题，并在短时间内实现了高精度。[12] 的作者提出了一种鲁棒的初始化框架，用于恢复单目相机的公制尺度，并估计外参变换和 IMU 偏置。Huang 等人 [13] 提出了一种由粗到细的方法，在线标定尺度因子、重力和外参变换。对于相机与惯性传感器之间时间偏移的标定，Mair 等人 [16] 提出了一种基于互相关和相位一致性分析的方法，并按照标准的手眼标定来标定外参旋转。Qin 等人 [17] 提出了一种在线方法，通过联合优化时间偏移、相机和 IMU 状态来标定时间偏移。

与视觉-惯性初始化相比，激光雷达-惯性系统的初始化研究要少得多。现有的一些激光雷达-惯性系统初始化方法要求传感器预先同步，或依赖额外传感器，或忽略某些初始状态。具体而言，Wang 等人 [18] 提出了一种在线初始化方法，用于估计激光雷达与 IMU 之间的时间和空间偏移，但需要相机作为额外的辅助传感器。类似地，[19] 使用 GPS/GNSS 来获取惯性传感器的精确位置和姿态。一些激光雷达-惯性里程计系统内置了初始化过程，但这些初始化模块通常简单且不完整。例如，[1] 初始化了陀螺仪偏置、重力向量和时间偏移。但该初始化相当粗糙。例如，时间偏移是通过假设传感器数据接收时间即采样时间来标定的，而数据传输和处理延迟被完全忽略，导致时间偏移估计不精确。[1] 中的陀螺仪偏置是通过在操作中让传感器保持静止几秒钟来标定的。由于设备静止时重力与加速度计偏置耦合，加速度计偏置在其初始化中未被标定。虽然 [1] 在线标定了外参，但外参初始化并未被考虑。因此，需要良好的初始猜测，否则后续激光雷达-惯性里程计的收敛性和鲁棒性将受到严重影响。与 [1] 类似，[20] 的初始化要求设备静止一段时间。加速度计偏置和激光雷达-惯性外参通过预先的离线标定获得，而时间偏移被假定为事先已知。与 [18]、[19] 相比，我们提出的方法不需要任何额外传感器。与 [1]、[20]、[21] 相比，我们的工作更为完整，它初始化了所有的时间偏移、外参、IMU 偏置和重力向量，并且对初始运动（如保持静止）或任何专门的时间同步或预标定没有任何特殊要求。

我们激光雷达-惯性初始化的主要目标之一，是在没有任何初始估计的情况下标定激光雷达与 IMU 之间的外参。一些现有的外参标定方法基于紧数据关联的批量优化，导致时间消耗大。例如，Lv 等人 [14] 提出了一种基于连续时间批量优化的标定方法。B 样条的使用导致需要估计更多参数，并带来较大的计算成本。[15] 使用扩展卡尔曼滤波器来估计外参变换，并伴随复杂的运动补偿，收敛速度有限。与这些方法相比，我们的方法更轻量，能够实时运行，同时仍能实现足够精确的外参标定，足以用于后续的在线估计（例如由 [1] 执行）。我们的方法还标定了 [14]、[15] 中未考虑的时间偏移。此外，[14]、[15] 采用的基于 NDT 的扫描到扫描匹配通常对非重复扫描模式的激光雷达效果不佳。相比之下，我们的方法采用扫描到地图匹配策略，可以轻松应用于重复扫描和非重复扫描的激光雷达。

---

## III. 方法

### A. 框架概览

由于 IMU 仅在运动时才会被激励 [15]，我们的初始化过程是一种基于运动的方法，这意味着充分的激励是必要的。我们工作流程的概览如图 2 所示，一些重要的符号列于表 I。我们提出的激光雷达里程计（见第 III-B 节）修改自 FAST-LIO2 [1]，采用恒定（角速度和线速度均为常数）速度（CV）模型来预测激光雷达运动并补偿一次扫描中的点畸变。为了缓解恒定速度模型与实际传感器运动之间的不匹配，我们将一个输入帧拆分为若干子帧，以提高激光雷达里程计的频率。如果激光雷达里程计没有失败（例如由于退化）并且所估计的激光雷达角速度和线速度满足我们提出的评估标准（见第 III-C.5 节），则认为激励充分，激光雷达里程计输出和相应的 IMU 数据都被送入初始化模块（见第 III-C 节）。在初始化中，首先通过平移 IMU 测量以与激光雷达里程计对齐来标定时间偏移，随后进行一次优化过程，进一步细化时间偏移、标定外参变换，并估计 IMU 偏置和重力向量。初始化后的状态可以输入到紧耦合的激光雷达-惯性里程计（例如 [1]），通过融合后续的激光雷达和 IMU 数据进行在线状态估计。

**图 2.** 我们的激光雷达-惯性初始化过程的框架。

**表 I**　一些重要符号

| 符号 | 说明 |
|---|---|
| $\boxplus$ / $\boxminus$ | 状态流形上封装的 "boxplus" 和 "boxminus" 运算 |
| $t_k$ | 第 $k$ 次激光雷达扫描的时间戳 |
| $\rho_j$ | 一次激光雷达扫描中第 $j$ 个点的时间戳 |
| $\tau_i$ | 第 $i$ 个 IMU 测量的时间戳 |
| $L_j$, $L_k$ | 在时间 $\rho_j$ 和 $t_k$ 时的激光雷达本体坐标系 |
| $x$, $\hat{x}$, $\bar{x}$ | 真实、预测和更新后的状态值 |
| $\check{x}$ | 反向传播中 $x_j$ 相对于 $x_k$ 的估计 |
| ${}^I R_L$, ${}^I p_L$ | 从激光雷达到 IMU 的外参旋转和平移 |
| ${}^I t_L$ | 激光雷达与 IMU 之间的总时间偏移 |
| $b_\omega$, $b_a$ | 陀螺仪和加速度计的偏置 |
| ${}^G g$ | 全局坐标系中的重力向量 |
| $I_i$, $I_k$ | 初始化步骤中使用的 IMU 数据序列，时间戳分别为 $\tau_i$、$t_k$ |
| $\bar{I}_k$ | 补偿已初始化时间偏移后的 IMU 数据序列，同步时间戳为 $t_k$ |
| $L_k$ | 初始化步骤中使用的激光雷达数据，时间戳为 $t_k$ |

### B. 激光雷达里程计

我们的纯激光雷达里程计与建图建立在恒定速度（CV）运动模型之上，该模型假设在 $t_k$ 和 $t_{k+1}$ 分别接收到的两个连续扫描之间，角速度和线速度是恒定的，即：

$$x_{k+1} = x_k \boxplus \big(\Delta t\, f(x_k, w_k)\big) \tag{1}$$

其中 $\Delta t$ 是两次扫描之间的时间间隔，状态向量 $x$、噪声 $w$ 和离散状态转移函数 $f$ 定义如下：

$$x = \begin{bmatrix} {}^G R_L \\ {}^G p_L \\ {}^G v_L \\ \omega_L \end{bmatrix},\quad w = \begin{bmatrix} n_v \\ n_\omega \end{bmatrix},\quad f(x, w) = \begin{bmatrix} \omega_L \\ {}^G v_L \\ n_v \\ n_\omega \end{bmatrix} \tag{2}$$

其中 ${}^G R_L \in SO(3)$、${}^G p_L$ 是激光雷达在全局坐标系（此处即第一个激光雷达本体坐标系 $L_0$）中的姿态和位置，${}^G v_L$ 是激光雷达在全局坐标系中描述的线速度，$\omega_L$ 是激光雷达在激光雷达本体坐标系中的角速度，它们被建模为由高斯噪声 $n_v$ 和 $n_\omega$ 分别驱动的随机游走过程。在式 (1) 中，我们使用 [22] 中定义的符号 $\boxplus$/$\boxminus$ 来紧凑地表示状态流形上的"加"运算。具体而言，对于式 (2) 中的状态流形 $SO(3) \times \mathbb{R}^n$，$\boxplus$ 运算及其逆运算 $\boxminus$ 定义如下：

$$\begin{bmatrix} R \\ a \end{bmatrix} \boxplus \begin{bmatrix} r \\ b \end{bmatrix} = \begin{bmatrix} R\,\mathrm{Exp}(r) \\ a + b \end{bmatrix};\qquad \begin{bmatrix} R_1 \\ a \end{bmatrix} \boxminus \begin{bmatrix} R_2 \\ b \end{bmatrix} = \begin{bmatrix} \mathrm{Log}(R_2^T R_1) \\ a - b \end{bmatrix}$$

其中 $R, R_1, R_2 \in SO(3)$，$r, a, b \in \mathbb{R}^n$，$\mathrm{Exp}(\cdot): \mathbb{R}^3 \mapsto SO(3)$ 是 $SO(3)$ 上的指数映射 [22]，$\mathrm{Log}(\cdot): SO(3) \mapsto \mathbb{R}^3$ 是其逆对数映射。

在实际中，传感器运动可能并不具有恒定速度。为了缓解这种模型误差的影响，我们可以将一次输入的激光雷达扫描拆分为持续时间更短的多个子帧，在这些子帧上传感器运动更符合 CV 模型。

#### 1) 误差状态迭代卡尔曼滤波

基于流形上的系统表示 (1)，我们使用误差状态迭代卡尔曼滤波器（ESIKF）[23] 来估计其状态。ESIKF 的预测步骤由状态预测和协方差传播组成，如下所示：

$$\hat{x}_{k+1} = \bar{x}_k \boxplus \big(\Delta t\, f(\bar{x}_k, 0)\big) \tag{3}$$

$$\hat{P}_{k+1} = F_{\tilde{x}} \bar{P}_k F_{\tilde{x}}^T + F_w Q F_w^T \tag{4}$$

其中 $P$、$Q$ 分别是状态估计和过程噪声 $w$ 的协方差矩阵。$F_{\tilde{x}}$ 和 $F_w$ 如下：

$$F_{\tilde{x}} = \frac{\partial \Big(\big(\bar{x}_k \boxplus \delta x_k\big) \boxplus \big(\Delta t\, f(\bar{x}_k \boxplus \delta x_k, 0)\big) \boxminus \big(\bar{x}_k \boxplus \Delta t\, f(\bar{x}_k, 0)\big)\Big)}{\partial \delta x_k}$$

$$F_{\tilde{x}} = \begin{bmatrix} \mathrm{Exp}\big(-\hat{\omega}_{L_k} \Delta t\big) & 0_{3\times3} & 0_{3\times3} & I_{3\times3}\,\Delta t \\ 0_{3\times3} & I_{3\times3} & I_{3\times3}\,\Delta t & 0_{3\times3} \\ 0_{3\times3} & 0_{3\times3} & I_{3\times3} & 0_{3\times3} \\ 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_{3\times3} \end{bmatrix}$$

$$F_w = \frac{\partial \Big(\bar{x}_k \boxplus \big(\Delta t\, f(\bar{x}_k, w_k)\big) \boxminus \big(\bar{x}_k \boxplus \Delta t\, f(\bar{x}_k, 0)\big)\Big)}{\partial w_k}$$

$$F_w = \begin{bmatrix} 0_{3\times3} & 0_{3\times3} \\ 0_{3\times3} & 0_{3\times3} \\ I_{3\times3}\,\Delta t & 0_{3\times3} \\ 0_{3\times3} & I_{3\times3}\,\Delta t \end{bmatrix} \tag{5}$$

其中 $\delta x$ 表示误差状态。

#### 2) 运动补偿

在我们所考虑的问题中，IMU 和激光雷达是不同步的，因此 [14]、[15] 所采用的 IMU 辅助运动补偿方法不可行。在时间戳 $t_{k+1}$ 接收到一次新的激光雷达扫描后，为了补偿运动畸变，我们将其中包含的每个点 ${}^{L_j} p_j$（在时间戳 $\rho_j \in (t_k, t_{k+1})$ 采样）投影到扫描结束时刻的激光雷达坐标系 $L_{k+1}$，如下所示。根据恒定速度模型，我们有 ${}^G \hat{v}_{L_{k+1}} = {}^G \bar{v}_{L_k}$、$\hat{\omega}_{L_{k+1}} = \bar{\omega}_{L_k}$，由此得到从时间 $\rho_j$ 到 $t_{k+1}$ 的相对变换 ${}^{L_{k+1}} \check{T}_{L_j} = ({}^{L_{k+1}} \check{R}_{L_j}, {}^{L_{k+1}} \check{p}_{L_j})$：

$${}^{L_{k+1}} \check{R}_{L_j} = \mathrm{Exp}\big(-\hat{\omega}_{L_{k+1}} \Delta t_j\big)$$

$${}^{L_{k+1}} \check{p}_{L_j} = -\, {}^G \hat{R}_{L_{k+1}}^T \, {}^G \hat{v}_{L_{k+1}}\, \Delta t_j \tag{6}$$

$$\Delta t_j = t_{k+1} - \rho_j.$$

然后，局部测量 ${}^{L_j} p_j$ 可以投影到扫描结束时刻的激光雷达坐标系：

$${}^{L_{k+1}} p_j = {}^{L_{k+1}} \check{T}_{L_j} \, {}^{L_j} p_j \tag{7}$$

随后，畸变补偿后的扫描 $\{ {}^{L_{k+1}} p_j \}$ 提供了未知状态 ${}^G T_{L_{k+1}}$ 的隐式测量，表示为点到平面距离残差，基于此在迭代卡尔曼滤波框架中迭代估计完整状态 $x_{k+1}$ 直到收敛。收敛后的状态估计（记为 $\bar{x}_{k+1}$）随后用于传播后续的 IMU 测量，如第 III-B.1 节所述。这种迭代估计的细节可参考 FAST-LIO2 [1]，或参考 [23] 对更一般的流形约束的处理。我们的激光雷达里程计使用有无运动补偿的扫描的建图对比如图 3 所示。

**图 3.** 使用 Livox Mid360 激光雷达的建图结果对比。(a) 无运动补偿的地图。(b) 使用按式 (6) 和 (7) 做畸变补偿后的扫描所构建的点云地图。(c, d) (b) 的建图细节。

### C. 激光雷达-惯性初始化

第 III-B 节的激光雷达里程计在每个扫描结束时刻（时间戳为 $t_k$）输出激光雷达的角速度 $\omega_{L_k}$ 和线速度 ${}^G v_{L_k}$。同时，IMU 提供原始测量，即本体角速度 $\omega_{m_i}$ 和线加速度 $a_{m_i}$（时间戳为 $\tau_i$）。这些数据被累积，并由第 III-C.5 节所示的激励标准反复评估。一旦收集到足够激励的数据，就调用初始化模块，最终输出时间偏移 ${}^I t_L \in \mathbb{R}$、外参 ${}^I T_L = ({}^I R_L, {}^I p_L) \in SE(3)$、IMU 偏置 $b_\omega, b_a \in \mathbb{R}^3$ 以及全局坐标系中的重力向量 ${}^G g \in \mathbb{R}^3$。

#### 1) 数据预处理

IMU 原始测量受到噪声 $n_{\omega_i}$ 和 $n_{a_i}$ 的影响。IMU 测量模型为：

$$\omega_{m_i} = \omega_i^{gt} + b_\omega + n_{\omega_i}, \qquad a_{m_i} = a_i^{gt} + b_a + n_{a_i} \tag{8}$$

其中 $\omega_i^{gt}$、$a_i^{gt}$ 是 IMU 角速度和线加速度的真实值。类似地，来自激光雷达里程计的估计 $\omega_{L_k}$、${}^G v_{L_k}$ 也包含噪声。

为了缓解这些通常是高频的噪声，使用非因果零相位低通滤波器 [24] 来滤除噪声而不引入任何滤波延迟。零相位滤波器通过前向和后向各运行一次巴特沃斯低通滤波器 [24] 来实现，产生降噪后的 IMU 测量 $\omega_{I_i} = \omega_i^{gt} + b_\omega$、$a_{I_i} = a_i^{gt} + b_a$。为符号简洁起见，降噪后的激光雷达估计仍记为 $\omega_{L_k}$、${}^G v_{L_k}$。

从激光雷达里程计 $\omega_{L_k}$、${}^G v_{L_k}$，我们通过非因果中心差分 [25] 获得激光雷达的角加速度和线加速度 $\Omega_{L_k}$、${}^G a_{L_k}$。所得的激光雷达里程计数据可以统一表示为：

$$L_k = \{\omega_{L_k}, {}^G v_{L_k}, \Omega_{L_k}, {}^G a_{L_k}\} \tag{9}$$

类似地，我们从降噪后的陀螺仪测量 $\omega_{I_i}$ 获得 IMU 角加速度 $\Omega_{I_i}$，得到：

$$I_i = \{\omega_{I_i}, a_{I_i}, \Omega_{I_i}\} \tag{10}$$

由于 IMU 频率通常高于激光雷达里程计，两个序列 $I_i$ 和 $L_k$ 大小不同。为了解决这个问题，我们提取同一时间段内接收的激光雷达和 IMU 数据，并在每个激光雷达里程计时刻 $t_k$ 对 $I_i$ 进行线性插值下采样（见图 4）。下采样后的 IMU 数据记为 $I_k$：

$$I_k = \{\omega_{I_k}, a_{I_k}, \Omega_{I_k}\} \tag{11}$$

它与 $L_k$ 具有相同的时间戳 $t_k$（但该数据实际上被已知的时间常数 ${}^I t_L$ 延迟）。

**图 4.** 通过在每个激光雷达里程计时间戳处进行插值来下采样 IMU 数据。

#### 2) 基于互相关的时间初始化

在大多数情况下，由于数据在被激光雷达-惯性里程计模块接收之前存在不可避免的传输和处理延迟，激光雷达 $L_k$ 与 IMU $I_k$ 之间会存在一个未知但恒定的偏移 ${}^I t_L$，使得 IMU 测量 $I_k$ 若被提前 ${}^I t_L$，将会与激光雷达里程计 $L_k$ 对齐。由于激光雷达数据 (9) 和 IMU 数据 (11) 都处于离散时刻 $t_k$，将 IMU 数据提前本质上是以离散步长 $d = {}^I t_L / \Delta t$ 进行的，其中 $\Delta t$ 是两次激光雷达扫描之间的时间间隔。具体而言，对于角速度，我们有：

$$\omega_{I_{k+d}} = {}^I R_L \omega_{L_k} + b_\omega \tag{12}$$

忽略通常很小的陀螺仪偏置 $b_\omega$，我们发现 $\omega_{I_{k+d}}$ 与 $\omega_{L_k}$ 的模长应当相同，而无论外参 ${}^I R_L$ 如何。受 [16] 启发，我们使用零中心互相关来量化二者模长之间的相似性。然后，偏移 $d$ 可以从以下优化问题中求解：

$$d^{*} = \arg\max_{d} \sum \|\omega_{I_{k+d}}\| \cdot \|\omega_{L_k}\| \tag{13}$$

通过在 $L_k$ 的索引范围内枚举偏移 $d$ 来求解。

#### 3) 联合外参旋转与时间标定

互相关方法对噪声和小尺度陀螺仪偏置具有鲁棒性。但式 (13) 的一个明显缺陷是，时间偏移的标定分辨率只能达到激光雷达里程计的一个采样间隔 $\Delta t$，任何小于 $\Delta t$ 的残余偏移 $\delta t$ 都无法被识别。设 ${}^I t_L$ 为激光雷达里程计 $\omega_L$ 与 IMU 数据 $\omega_I$ 之间的总偏移，则 ${}^I t_L = d^{*} \Delta t + \delta t$。类似于式 (12)，若 IMU 测量 $\omega_I$ 被提前时间 ${}^I t_L$，将与激光雷达里程计 $\omega_L$ 对齐：

$$\omega_I(t + {}^I t_L) = {}^I R_L \omega_L(t) + b_\omega \tag{14}$$

由于式 (9) 中的实际激光雷达里程计 $\omega_L$ 仅在时间戳 $t_k$ 可用，将 $t = t_k$ 和 ${}^I t_L = d^{*} \Delta t + \delta t$ 代入式 (14)，并注意到 $\omega_L(t_k) = \omega_{L_k}$，我们有：

$$\omega_I(t_k + d^{*} \Delta t + \delta t) = {}^I R_L \omega_{L_k} + b_\omega \tag{15}$$

注意到 $\omega_I(t_k + d^{*} \Delta t + \delta t)$ 是时间 $t_k + d^{*} \Delta t$ 之后的 IMU 角速度，此处的角速度和角加速度分别为 $\omega_I(t_k + d^{*} \Delta t) = \omega_{I_{k'}}$ 和 $\Omega_I(t_k + d^{*} \Delta t) = \Omega_{I_{k'}}$，其中 $k' = k + d^{*}$。我们可以通过假设角加速度在小的 $\delta t$ 内恒定来插值 $\omega_I(t_k + d^{*} \Delta t + \delta t)$ 的值（见图 5）：

$$\omega_I(t_k + d^{*} \Delta t + \delta t) \approx \omega_{I_{k'}} + \delta t\, \Omega_{I_{k'}} \tag{16}$$

可将其代入式 (15) 得到：

$$\omega_{I_{k'}} + \delta t\, \Omega_{I_{k'}} = {}^I R_L \omega_{L_k} + b_\omega \tag{17}$$

最后，基于式 (17) 中的约束，统一的时间-空间优化问题可以表述为：

$$\arg\min_{{}^I R_L,\, b_\omega,\, \delta t} \sum \Big\|{}^I R_L \omega_{L_k} + b_\omega - \omega_{I_{k'}} - \delta t \cdot \Omega_{I_{k'}}\Big\|^2 \tag{18}$$

由于非线性约束 ${}^I R_L \in SO(3)$，该问题由 Ceres Solver² 从初始值 $({}^I R_L, b_\omega, \delta t) = (I_{3\times3}, 0_{3\times1}, 0)$ 开始迭代求解。

**图 5.** 时间偏移标定以及式 (16) 所示一阶近似的示意图。

#### 4) 外参平移与重力初始化

在第 III-C.3 节中，我们获得了外参旋转 ${}^I R_L$、陀螺仪偏置 $b_\omega$ 和时间偏移 ${}^I t_L$。在本节中，我们固定这些值，继续进行外参平移、重力向量和加速度计偏置的标定。

首先，我们使用先前标定的偏移 $d^{*}$ 和残余 $\delta t$ 将 IMU 数据 $I_k$ 与激光雷达数据 $L_k$ 对齐。对齐后的 IMU 数据记为 $\bar{I}_k$，此时假定其与 $L_k$ 完全对齐、无时间偏移。具体而言，在时间 $t_k$ 与激光雷达角速度 $\omega_{L_k}$ 对应的 IMU 角速度 $\bar{\omega}_{I_k}$ 为（见式 (15)）：

$$\bar{\omega}_{I_k} = \omega_I(t_k + d^{*} \Delta t + \delta t) \approx \omega_{I_{k+d}} + \delta t\, \Omega_{I_{k+d}} \tag{19}$$

类似地，在时间 $t_k$ 与激光雷达加速度 ${}^G a_{L_k}$ 对应的 IMU 加速度 $\bar{a}_{I_k}$ 为：

$$\bar{a}_{I_k} = a_I(t_k + d^{*} \Delta t + \delta t) \approx a_{I_{k+d}} + \frac{\delta t}{\Delta t}\big(a_{I_{k+d+1}} - a_{I_{k+d}}\big) \tag{20}$$

然后，类似于式 (14)，我们可以找到 IMU 与激光雷达之间的加速度约束。如 [26] 所述，具有固定外参的两个坐标系 $A$、$B$ 的加速度具有如下关系：

$${}^A R_B\, a_B = a_A + \lfloor \omega_A \rfloor_\wedge^2 \, {}^A p_B + \lfloor \Omega_A \rfloor_\wedge \, {}^A p_B \tag{21}$$

其中 ${}^A R_B$、${}^A p_B$ 表示从坐标系 $B$ 到坐标系 $A$ 的外参变换。$a_A$、$a_B$ 都在各自的本体坐标系中描述。

对于激光雷达-惯性系统，我们有两种选择：$A$ 为 IMU、$B$ 为激光雷达，或者相反。注意到在第一种情况下，$\omega_A = \bar{\omega}_{I_k} - b_\omega$ 的精度受陀螺仪偏置估计的影响，并且由于角速度测量中的噪声，$\Omega_A$ 的误差会被放大。为了避免这个问题并提高外参平移标定的鲁棒性，我们设激光雷达为 $A$、IMU 为 $B$。由于激光雷达的加速度 ${}^G a_{L_k}$ 是在全局坐标系中描述的，我们需要计算激光雷达在本体坐标系中描述的瞬时加速度，记为 $a_{L_k}$：

$$a_{L_k} = {}^G R_L^T \big({}^G a_{L_k} - {}^G g\big) \tag{22}$$

其中 ${}^G R_L$ 是激光雷达在全局坐标系中的姿态，由第 III-B 节的激光雷达里程计获得。

最后，外参平移、加速度计偏置和重力向量可以从以下优化问题联合估计：

$$\arg\min_{{}^I p_L,\, b_a,\, {}^G g} \sum \Big\|{}^I R_L^T \big(\bar{a}_{I_k} - b_a\big) - a_{L_k} - \big(\lfloor \omega_{L_k} \rfloor_\wedge^2 + \lfloor \Omega_{L_k} \rfloor_\wedge\big) {}^L p_I\Big\|^2 \tag{23}$$

由于约束 ${}^G g \in S^2$，该问题由 Ceres Solver 从初始值 $({}^I p_L, b_a, {}^G g) = (0_{3\times1}, 0_{3\times1}, 9.81\,e_3)$ 开始迭代求解。在估计出 ${}^L p_I$ 之后，从激光雷达到 IMU 的平移可以计算为 ${}^I p_L = -{}^I R_L \, {}^L p_I$。

#### 5) 数据累积评估

所提出的初始化方法依赖于激光雷达-惯性设备的充分激励（足够的运动）。因此，系统应当能够自行评估激励是否足以执行初始化。理想情况下，激励可以通过式 (18) 关于 $({}^I R_L, b_\omega, \delta t)$ 以及式 (23) 关于 $({}^I p_L, b_a, {}^G g)$ 的完整雅可比矩阵的秩来评估。在实践中，我们发现仅评估关于外参旋转 ${}^I R_L$ 和外参平移 ${}^I p_L$ 的雅可比就足够了，因为对外参的激励通常需要激发其他状态的复杂运动。记 $J_r$ 为式 (18) 关于 ${}^I R_L$ 的雅可比，$J_t$ 为式 (23) 关于 ${}^I p_L$ 的雅可比，

$$J_r = \begin{bmatrix} \vdots \\ -{}^I R_L \lfloor \omega_{L_k} \rfloor_\wedge \\ \vdots \end{bmatrix},\qquad J_t = \begin{bmatrix} \vdots \\ \lfloor \omega_{L_k} \rfloor_\wedge^2 + \lfloor \Omega_{L_k} \rfloor_\wedge \\ \vdots \end{bmatrix} \tag{24}$$

然后，激励可以从 $J_r^T J_r = \sum \lfloor \omega_{L_k} \rfloor_\wedge^T \lfloor \omega_{L_k} \rfloor_\wedge$ 和 $J_t^T J_t = \sum \big(\lfloor \omega_{L_k} \rfloor_\wedge^2 + \lfloor \Omega_{L_k} \rfloor_\wedge\big)^T \big(\lfloor \omega_{L_k} \rfloor_\wedge^2 + \lfloor \Omega_{L_k} \rfloor_\wedge\big)$ 的秩来评估。更定量地说，激励程度由 $J_r^T J_r$ 和 $J_t^T J_t$ 的奇异值指示。基于这一原理，我们开发了一个评估程序，可以指导用户如何移动他们的设备以获得充分的激励。我们基于雅可比矩阵的奇异值来量化激励，并设置一个阈值来评估激励是否充分。

---

## IV. 实验

我们主要在自行组装的激光雷达-惯性手持设备（图 1）所采集的数据集上评估我们的初始化方法。我们使用多种类型的激光雷达（Livox³ Avia/Mid360 和 Hesai PandarXT⁴）以及 6 轴 IMU（Pixhawk 飞控⁵和 Livox 激光雷达内部的 Bosch BMI088）来测试我们的初始化算法。激光雷达的原始数据频率设置为 10 Hz（每秒 10 次扫描），IMU 原始数据频率为 200 Hz。所有实验都在配备 Intel i7-10700 @2.90 GHz、32 GB RAM 的台式计算机上进行。在所有实验中，式 (18) 的初始状态设置为 $({}^I R_L, b_\omega, \delta t) = (I_{3\times3}, 0_{3\times1}, 0)$，式 (23) 的初始状态设置为 $({}^I p_L, b_a, {}^G g) = (0_{3\times1}, 0_{3\times1}, 9.81\,e_3)$。其他状态的初始化不需要任何初始值。

### A. 时间初始化评估

为了展示我们时间初始化（标定）方法的有效性和准确性，我们在 Livox 激光雷达（Livox Avia 和 Livox Mid360）及其内置 IMU 所采集的数据上进行了测试。由于每个 Livox 激光雷达与其内置 IMU 在出厂时已经硬件同步，真实时间偏移约为 0 秒，精度达到微秒级。为了展示我们时间标定的能力，我们手动平移输入 IMU 时间戳来构造人工时间偏移 [17]，即向 IMU 时间戳添加一个固定值 ${}^I t_L$。我们在实验室场景中使用每个 Livox 激光雷达及其内置 IMU 采集了 5 个数据序列。标定结果如表 II 所示。可以看出，时间标定误差处于微秒级，足以满足大多数激光雷达-惯性传感器融合算法的要求。结果表明，我们的时间标定方法具有很高的精度和一致性。

**表 II**　时间初始化结果

| 激光雷达 | ${}^I t_L$ (s) | 均值 [s] | RMSE [s] | NEES |
|---|---|---|---|---|
| Avia | 0.05 | 0.0490 | 0.0016 | 3.2% |
|  | 0.1 | 0.0988 | 0.0017 | 1.7% |
|  | 0.5 | 0.4989 | 0.0018 | 0.36% |
| Mid360 | 0.05 | 0.0479 | 0.0034 | 6.8% |
|  | 0.1 | 0.0982 | 0.0028 | 2.8% |
|  | 0.5 | 0.4984 | 0.0029 | 0.58% |

> Livox 激光雷达及其内置 IMU 的时间初始化结果。RMSE 表示均方根误差，NEES 表示已标定时间偏移的归一化估计误差平方 [17]，计算为 $\mathrm{NEES} = \mathrm{RMSE} / {}^I t_L$。

对于不同步的自组装激光雷达-惯性系统，真实时间偏移难以获得。一种能够验证我们方法有效性的间接方式，是考察紧耦合激光雷达-惯性里程计（LIO）在使用标定后的时间偏移时的性能。我们采用的 LIO 是 FAST-LIO2 [1]，一种最先进的激光雷达惯性里程计，其内部提供时间同步和状态初始化。我们在不同步的 Hesai PandarXT 激光雷达与 Pixhawk IMU（图 1）所采集的数据序列上测试 FAST-LIO2，分别使用其自身标定的时间偏移和我们方法标定的时间偏移，同时保持其他状态初始化不变（外参来自 CAD 参考）。定位和建图结果如图 6 所示。可以看出，使用我们时间标定的整体 LIO 性能优于使用 LIO 内部时间同步的性能。我们的方法在 11.3627 m 的轨迹上端到端漂移仅为 0.0102 m，而 LIO 内部时间同步为 0.246 m；我们方法所建地图也保留了更精细的结构细节。

**图 6.** 使用 FAST-LIO2 在同一实验室采集数据集上的建图结果对比（鸟瞰图）。设备来回摆动以获得充分的激励，并返回原点。整个序列持续 55.8 秒，其中前 20 秒用于标定时间偏移。我们使用标定后的偏移在整个序列上重新运行 FAST-LIO2。(a) FAST-LIO2 中带时间同步的建图结果，地图精度受其内部同步不准确的限制。

### B. 空间初始化评估

#### 1) 多种激光雷达类型测试

我们在图 1 所示平台采集的数据序列上测试我们的空间（外参）初始化方法。由于激光雷达被外壳覆盖，而 IMU 内置在 Pixhawk 飞控中，它们精确的外参难以直接获知。然而，我们可以通过一种间接方式来验证我们的外参初始化方法。我们将 Pixhawk 固定在图 1 所示的两个位姿 $I_1$ 和 $I_2$ 处，它们的真实相对位姿在 CAD 中设计并在实际装配中保证。真实相对位姿为：旋转欧拉角 $(0, 0, 0)$ 度，平移 $(0.25, 0, 0)$ 米；CAD 的制造精度可达 0.01 度和毫米级。在每个位姿下，标定激光雷达与 IMU 的外参，分别记为 ${}^{I_1} T_L$、${}^{I_2} T_L$。然后相对位姿计算为 ${}^{I_1} T_{I_2} = {}^{I_1} T_L \, {}^{I_2} T_L^{-1}$，可与 CAD 参考提供的真值进行比较。为了证明我们的标定方法支持具有不同扫描模式的多种激光雷达，我们为每种激光雷达采集 5 个数据序列，包括 Livox Mid360、Livox Avia 和 Hesai PandarXT，并让 Pixhawk IMU 分别处于两个位姿。

在所有实验中，我们将初始外参设置为 $({}^I R_L, {}^I p_L) = (I_{3\times3}, 0_{3\times1})$，尽管真值与初始值相差甚远（例如，对于 Mid360，当 IMU 在 $I_1$ 处时，实际外参旋转约为 $(0, -2, 178)$ 度，平移约为 $(0.12, 0, 0.11)$ 米）。我们计算相对 IMU 位姿误差绝对值的平均值和标准差。结果如表 III 所示。可以看出，平移误差处于厘米级，旋转误差小于 $1^\circ$。整体较小的误差表明，在相同传感器设置的不同数据集上，外参初始化结果彼此接近，可作为后续激光雷达-惯性里程计的高质量初始值。

**表 III**　外参初始化结果

| 激光雷达 | Livox Mid360 | Livox Avia | PandarXT |
|---|---|---|---|
| 相对误差　Rot($^\circ$) | $0.2472 \pm 0.2043$ | $0.4019 \pm 0.1708$ | $0.7244 \pm 0.5076$ |
| 　Trans(m) | $0.0081 \pm 0.0075$ | $0.0064 \pm 0.0069$ | $0.0133 \pm 0.0102$ |

> 外参初始化的均值与 SD（标准差）。

表 III 中另一个有趣的现象是，Livox Avia 和 Livox Mid360 的外参标定误差小于 Hesai PandarXT。这主要得益于 Livox 激光雷达的非重复扫描。由于我们的激光雷达里程计基于恒定速度（CV）模型，输入的激光雷达帧根据采样时间被拆分为若干子帧，以提高里程计频率，从而缓解 CV 模型的不匹配。对于非重复扫描的激光雷达，帧拆分不会改变子帧的视场。此外，当激光雷达保持静止时，点图会变得更密集，这有利于我们的扫描到地图策略。相比之下，对于 Hesai PandarXT 等机械旋转激光雷达，帧拆分会减小子帧的视场并降低激光雷达里程计的鲁棒性。因此，我们在采集数据时不能快速移动，这降低了 IMU 激励和信噪比（SNR）。

#### 2) 精度与鲁棒性对比

为了进一步验证我们外参初始化的精度，我们选择最先进的激光雷达-惯性外参标定方法进行对比，即 LI-Calib [14] 和 Target-Free [15]。由于这两种方法仅支持机械旋转激光雷达，我们选择第 IV-B.1 节中由 Hesai PandarXT 和 Pixhawk 采集的两个序列，并使用相同的相对 IMU 位姿误差来评估标定精度。图 7(e) 显示了这两个序列的场景。我们使用前 40 秒（包括 400 次激光雷达扫描和相应的 IMU 测量）运行所有标定方法。由于 LI-Calib 和 Target-Free 无法进行时间标定，在测试它们时时间偏移被预先补偿。两个序列上均使用这两种方法的默认参数。时间消耗和平均相对 IMU 位姿误差如表 IV 所示。可以看出，我们的方法更准确且耗时少得多。图 7 展示了一些定性结果。LI-Calib [14] 采用由粗到精的标定。在粗标定中，它通过 NDT 匹配配准激光雷达扫描并标定外参旋转；在精标定阶段，它执行所有外参参数的批量优化。粗标定所构建的地图如图 7(a) 所示。LI-Calib 的精标定由于忽略了重力初始化而失败，导致混乱的 surfel 地图，如图 7(b) 所示。相比之下，我们标定方法中的纯激光雷达里程计鲁棒且准确，其点图如图 7(c, d) 所示。Target-Free [15] 不提供可视化模块来显示其地图。这些结果表明，我们的方法比 [14] 具有更好的鲁棒性，与 [15] 相比精度相当，而计算成本远低于 [14]、[15]。

**图 7.** (a) LI-Calib [14] 粗标定期间构建的 surfel 地图。(b) LI-Calib 的精标定失败，surfel 地图一片混乱。(c, d) 我们的标定期间构建的精确点云地图，见第 III-B 节。(e) 标定场景。

**表 IV**　外参标定对比

| 方法 | 旋转 ($^\circ$) | 平移 (m) | 时间 (s) |
|---|---|---|---|
| Proposed | 0.6208 | 0.0162 | 10.2 |
| LI-Calib [14] | 1.0375 | × | 332.6 |
| Target-Free [15] | 0.8483 | 0.0187 | 115.7 |

> 数据集的前 40 秒用于标定。× 表示 LI-Calib [14] 的精标定失败，由于忽略重力向量初始化，标定结果发散。因此我们使用其粗标定中得到的旋转。

### C. 时间消耗评估

与 [14]、[15] 这两种计算负荷高、无法实时处理的外参标定方法相比，我们的方法快速且能够实时实现。与使用批量优化最小化点到平面残差不同，我们的激光雷达里程计非常高效，一个子帧的平均处理时间约为 8 ms。一旦收集到足够的数据，初始化求解器的总时间消耗（包括数据预处理、时间初始化、外参和重力初始化）小于 500 ms，远小于实际的数据采集时间。

为了评估效率，我们在同一公寓走廊采集的数据集上测试 [14]、[15] 和我们的方法，并比较每次扫描运动补偿的时间消耗。此外，我们还比较了数据长度（以激光雷达输入大小衡量）不同时的总标定时间。所有结果如图 9 所示，这表明我们的方法具有很高的计算效率。此外，随着输入数据量的增加，我们方法的处理时间增长缓慢。

**图 9.** 时间消耗对比。所有测试数据均由 10 Hz 输出的 Hesai PandarXT 激光雷达采集。蓝色区域是初始化时间低于数据采集时间的区域，因此可以实时运行。

### D. 重力与偏置初始化评估

我们的方法能够标定陀螺仪偏置、加速度计偏置和重力向量，这些可作为实时激光雷达-惯性里程计系统的高质量初始状态。为了检验我们方法的精度，我们将其集成到 FAST-LIO2 [1] 中，后者通过紧融合后续的激光雷达数据与 IMU 来进一步细化所有状态，包括偏置和重力向量。我们在图 8 中绘制了 FAST-LIO2 在线估计的状态与我们的初始化所提供的初始值之间的差异。可以看出，初始的陀螺仪偏置、加速度计偏置和重力向量已经非常准确，后续的细化幅度很小。

**图 8.** 陀螺仪偏置、加速度计偏置和重力向量经 FAST-LIO2 细化后与其初始值（由我们的初始化提供）之间的误差。测试数据集由 Livox Mid360 激光雷达和 Pixhawk 飞控内的 IMU 采集。

---

## V. 结论

本文提出了一种快速、鲁棒的激光雷达-惯性系统时间与空间初始化方法。针对不同步的激光雷达和 IMU，我们提出了一种精确、高效的由粗到细的时间标定方法，该方法不依赖于任何硬件同步设置。此外，我们提出了一种快速、新颖的数据关联函数，用于初始化激光雷达-惯性外参变换、重力向量以及陀螺仪和加速度计的偏置。各种实验表明，我们的初始化方法具有一致性、鲁棒性和高质量。此外，使用多种类型激光雷达的实验证明了该方法对具有不同扫描模式的激光雷达的适用性。

---

## VI. 致谢

本工作部分由香港大学教育资助委员会研究资助局（项目 17206421）资助，部分由 DJI（资助号 200009538）资助。作者衷心感谢 Livox Technology 在整个工作期间提供的设备支持。作者感谢 Wei Xu、Yixi Cai、Jiajun Lv、Meng Li 的有益讨论与支持。

---

## 脚注

1. https://www.github.com/hku-mars/LiDAR_IMU_Init
2. http://ceres-solver.org/
3. https://www.livoxtech.com
4. https://www.hesaitech.com/en/PandarXT
5. https://ardupilot.org/copter/docs/common-cuav-nora-overview.html

---

## 参考文献

（以下为原文，未翻译）

[1] Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, and Fu Zhang. Fast-lio2: Fast direct lidar-inertial odometry. IEEE Transactions on Robotics, 2022.

[2] Jiarong Lin and Fu Zhang. Loam livox: A fast, robust, high-precision lidar odometry and mapping package for lidars of small fov. In 2020 IEEE International Conference on Robotics and Automation (ICRA), pages 3126–3131. IEEE, 2020.

[3] Zheng Liu and Fu Zhang. Balm: Bundle adjustment for lidar mapping. IEEE Robotics and Automation Letters, 6(2):3184–3191, 2021.

[4] Yunfan Ren, Fangcheng Zhu, Wenyi Liu, Zhepei Wang, Yi Lin, Fei Gao, and Fu Zhang. Bubble planner: Planning high-speed smooth quadrotor trajectories using receding corridors. arXiv preprint arXiv:2202.12177, 2022.

[5] Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza. On-manifold preintegration for real-time visual–inertial odometry. IEEE Transactions on Robotics, 33(1):1–21, 2016.

[6] Michael Bloesch, Michael Burri, Sammy Omari, Marco Hutter, and Roland Siegwart. Iterated extended kalman filter based visual-inertial odometry using direct photometric feedback. The International Journal of Robotics Research, 36(10):1053–1072, 2017.

[7] Tixiao Shan, Brendan Englot, Drew Meyers, Wei Wang, Carlo Ratti, and Daniela Rus. Lio-sam: Tightly-coupled lidar inertial odometry via smoothing and mapping. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 5135–5142. IEEE, 2020.

[8] Kailai Li, Meng Li, and Uwe D Hanebeck. Towards high-performance solid-state-lidar-inertial odometry and mapping. IEEE Robotics and Automation Letters, 6(3):5167–5174, 2021.

[9] Jiarong Lin, Chunran Zheng, Wei Xu, and Fu Zhang. R2 live: A robust, real-time, lidar-inertial-visual tightly-coupled state estimator and mapping. IEEE Robotics and Automation Letters, 6(4):7469–7476, 2021.

[10] Jiarong Lin and Fu Zhang. R3 live: A robust, real-time, rgb-colored, lidar-inertial-visual tightly-coupled state estimation and mapping package. In 2022 International Conference on Robotics and Automation (ICRA), pages 10672–10678. IEEE, 2022.

[11] Raúl Mur-Artal and Juan D Tardós. Visual-inertial monocular slam with map reuse. IEEE Robotics and Automation Letters, 2(2):796–803, 2017.

[12] Tong Qin and Shaojie Shen. Robust initialization of monocular visual-inertial estimation on aerial robots. In 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 4225–4232. IEEE, 2017.

[13] Weibo Huang and Hong Liu. Online initialization and automatic camera-imu extrinsic calibration for monocular visual-inertial slam. In 2018 IEEE International Conference on Robotics and Automation (ICRA), pages 5182–5189. IEEE, 2018.

[14] Jiajun Lv, Jinhong Xu, Kewei Hu, Yong Liu, and Xingxing Zuo. Targetless calibration of lidar-imu system based on continuous-time batch estimation. In 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 9968–9975. IEEE, 2020.

[15] Subodh Mishra, Gaurav Pandey, and Srikanth Saripalli. Target-free extrinsic calibration of a 3d-lidar and an imu. In 2021 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI), pages 1–7. IEEE, 2021.

[16] Elmar Mair, Michael Fleps, Michael Suppa, and Darius Burschka. Spatio-temporal initialization for imu to camera registration. In 2011 IEEE International Conference on Robotics and Biomimetics, pages 557–564. IEEE, 2011.

[17] Tong Qin and Shaojie Shen. Online temporal calibration for monocular visual-inertial systems. In 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pages 3662–3669. IEEE, 2018.

[18] Yan Wang and Hongwei Ma. Online spatial and temporal initialization for a monocular visual-inertial-lidar system. IEEE Sensors Journal, 2021.

[19] Zachary Taylor and Juan Nieto. Motion-based calibration of multimodal sensor arrays. In 2015 IEEE International Conference on Robotics and Automation (ICRA), pages 4843–4850. IEEE, 2015.

[20] Chao Qin, Haoyang Ye, Christian E Pranata, Jun Han, Shuyang Zhang, and Ming Liu. Lins: A lidar-inertial state estimator for robust and efficient navigation. In 2020 IEEE International Conference on Robotics and Automation (ICRA), pages 8899–8906. IEEE, 2020.

[21] Wei Xu and Fu Zhang. Fast-lio: A fast, robust lidar-inertial odometry package by tightly-coupled iterated kalman filter. IEEE Robotics and Automation Letters, 6(2):3317–3324, 2021.

[22] C. Hertzberg, R. Wagner, U. Frese, and L. Schröder. Hertzberg, christoph and wagner, rené and frese, udo and schröder, lutz. Information Fusion, 14(1):57–77, 2013.

[23] Dongjiao He, Wei Xu, and Fu Zhang. Embedding manifold structures into kalman filters. arXiv preprint arXiv:2010.05957, 2021.

[24] Fredrik Gustafsson. Determining the initial states in forward-backward filtering. IEEE Transactions on signal processing, 44(4):988–992, 1996.

[25] Dumitru Baleanu, Ozlem Defterli, and Om P. Agrawal. A central difference numerical scheme for fractional optimal control problems. Journal of Vibration and Control, 15(4):583–597, 2009.

[26] Wei Xu, Dongjiao He, Yixi Cai, and Fu Zhang. Robots' state estimation and observability analysis based on statistical motion models. IEEE Transactions on Control Systems Technology, 2022.
