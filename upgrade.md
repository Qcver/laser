# Point-LIO 源码优化

> **范围**：`~/point_lio/src/9000_point_lio_ros2_Mid-360`（上游 `9000_point_lio_ros2_Mid-360` fork）
> **改动规模**：6 个文件，+79 / −22 行（`config/mid360.yaml` 未改动，那是此前的人工调参）

## 目录

| # | 优化项                                         | 类型                     | 文件                                                                              | 收益                                                  |
| - | ---------------------------------------------- | ------------------------ | --------------------------------------------------------------------------------- | ----------------------------------------------------- |
| 1 | `blind` 判据括号优先级 bug                   | 正确性                   | `src/preprocess.cpp`                                                            | 盲区过滤从"基本失效"变为生效                          |
| 2 | **点时间戳被错误改写**                   | **正确性（严重）** | `src/preprocess.cpp`                                                            | 单帧耗时 **221 ms → 3.8 ms**；恢复逐点运动补偿 |
| 3 | iEKF 迭代次数可配 + 协方差更新时机             | 能力 + 正确性            | `include/IKFoM/.../esekfom.hpp`、`src/parameters.*`、`src/laserMapping.cpp` | 可调精度/算力；避免协方差过度收缩                     |
| 4 | 启动自检日志                                   | 可观测性                 | `src/laserMapping.cpp`                                                          | 一眼核对实际生效参数                                  |
| 5 | 参数调整（`point_filter_num`、`max_iter`） | 调参                     | `launch/mapping_mid360.launch.py`                                               | 见第 5 节                                             |

---

# 1. 修复 `blind` 判据

**文件**：[`src/preprocess.cpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/preprocess.cpp) `Preprocess::avia_handler()`，131–145 行

## 1.1 问题

```cpp
// 修复前
if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7)
    || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7)
    || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)
       && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y
           + pl_full[i].z * pl_full[i].z > (blind * blind))) {
    pl_surf.push_back(pl_full[i]);
}
```

C++ 中 `&&` 的优先级高于 `||`，实际语义是：

```
(dx > eps)  OR  (dy > eps)  OR  (dz > eps AND range² > blind²)
```

→ **只要 x 或 y 变化（几乎总成立），近距点照样保留**。`blind`（本项目 0.1 m）实际上只对"x、y 都没变、只有 z 变了"的点生效，等于失效。

这个写法继承自上游 FAST-LIO，其它分支（velodyne / unilidar / hesai）用的是正确的纯半径判据，只有 Livox 分支有这个问题。

## 1.2 修复

```cpp
// 修复后（preprocess.cpp:132-145）
// 丢弃盲区内的点（与 velodyne/unilidar 分支保持同一判据），
// 同时保留"与上一个保留点完全相同则跳过"的去重逻辑。
const float range2 = pl_full[i].x * pl_full[i].x +
                     pl_full[i].y * pl_full[i].y +
                     pl_full[i].z * pl_full[i].z;
const bool moved =
    (std::fabs(pl_full[i].x - pl_full[i - 1].x) > 1e-7f) ||
    (std::fabs(pl_full[i].y - pl_full[i - 1].y) > 1e-7f) ||
    (std::fabs(pl_full[i].z - pl_full[i - 1].z) > 1e-7f);
if (moved && range2 > blind * blind) {
    pl_surf.push_back(pl_full[i]);
}
```

同时：

- 新增 `#include <cmath>`；
- `abs()` → `std::fabs()`，避免整型/浮点重载歧义。

## 1.3 验证

把新旧判据抽成独立小程序，7 组用例全部符合预期：

| 用例               | 修复前 | 修复后         | 期望 |
| ------------------ | ------ | -------------- | ---- |
| 0.05 m，x 变化     | KEEP   | **DROP** | DROP |
| 0.05 m，仅 z 变化  | DROP   | DROP           | DROP |
| 0.20 m             | KEEP   | KEEP           | KEEP |
| 3.00 m             | KEEP   | KEEP           | KEEP |
| 坐标完全重复       | DROP   | DROP           | DROP |
| 0.0999 m（盲区内） | KEEP   | **DROP** | DROP |
| 0.1001 m（盲区外） | KEEP   | KEEP           | KEEP |

## 1.4 影响评估

- **对本项目几乎无感**：Mid-360 硬件近距盲区本来就是 0.1 m，所以"漏掉的近距点"大多本来就没有数据。
- **真正受益的场景**：雷达贴车体安装、或视场里有自车结构件时，近距离返点不会再进入地图造出假平面。
- **风险**：去除的是"近距无效点"，不影响正常点。若雷达前方 10 cm 内有真实目标（如抓取任务贴近物体），该目标的点会被丢弃——这是 `blind` 的**设计意图**，不是本次改动引入的问题。

---

# 2. 修复点时间戳分组

**文件**：[`src/preprocess.cpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/preprocess.cpp) `Preprocess::avia_handler()`，100–129 行

## 2.1 问题

```cpp
// 修复前
pl_full[i].curvature = msg->points[i].offset_time / float(1000000);  // 帧内偏移

if (i == 0) pl_full[i].curvature = fabs(pl_full[i].curvature) < 1.0 ? pl_full[i].curvature : 0.0;
else pl_full[i].curvature = 
  fabs(pl_full[i].curvature - pl_full[i - 1].curvature) < 1.0 ? pl_full[i].curvature
                                               : pl_full[i - 1].curvature + 0.004166667f;
```

这段"时间戳连续性检查"是上游 FAST-LIO 的写法，**隐含假设 `point_filter_num == 1`**（即相邻两点都被保留、`pl_full[i-1].curvature` 一定有值）。

而本项目 `point_filter_num = 3`：

1. `pl_full` 是 `resize(plsize)` 出来的，**未赋值的索引 curvature 恒为 0**；
2. 循环里只有 `valid_num % 3 == 0` 的点被写入，所以 `pl_full[i-1]` 有 **2/3 的概率是 0**；
3. 于是 `|curv - 0| >= 1.0ms`（帧内偏移几乎都 ≥1 ms，100 ms 内分布）→ 走 else 分支
   → `0 + 0.004166667 ms` = **整帧所有点的 curvature 都变成 0.0041667**；
4. 另注：`if (i == 0)` 分支位于 `for (uint i = 1; ...)` 循环内，**永远不会执行**（死代码）。

Python 复现该逻辑的确切结果（9600 点/帧、pf=3）：

```
keep=3199  groups=32  max_group=3168   ← 3168 个点落进同一个时间组
```

## 2.2 连锁影响（比"精度差一点"严重）

| #  | 后果                                       | 说明                                                                                                                                                       |
| -- | ------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ① | **逐点更新退化为"整帧一次更新"**     | `time_compressing` 按相同 curvature 分组，所有点被当作在 `t≈0` 时刻测得，帧内 100 ms 的运动补偿完全丢失——这正是 Point-LIO 相对 FAST-LIO2 的核心优势 |
| ② | **求解复杂度 O(N) → O(N³)**        | 单组点数 N≈3168，量测矩阵变成 N×N，`HPHT.inverse()` 立方增长                                                                                           |
| ③ | **`lidar_end_time` 早了约 100 ms** | `lidar_end_time = lidar_beg_time + max(curvature)/1000`，于是 odom 时间戳整体提前，下游用 `now()` 查 TF 会看到约 100 ms 滞后                           |
| ④ | **批次线性化误差**                   | 整帧共用一次 EM 线性化点，产生随运动增大的系统性偏差（见 2.4 中被撤回的旧结论）                                                                            |

## 2.3 修复

```cpp
// 新增状态（preprocess.cpp:103-104）
float prev_kept_curv = 0.0f;
bool has_prev_kept = false;

// 修复后（preprocess.cpp:116-129）
// curvature = 该点相对帧头的时间（ms）。以"上一个被保留的点"为参考做连续性检查：
// 原实现取 pl_full[i-1]，但 point_filter_num > 1 时该索引大多未被赋值（=0），
// 导致帧内偏移 ≥ 1 ms 的点全部退化成同一个 0.0041667 ms，
// 整帧在 time_compressing 里塌缩成一个时间组：
//   · 丢失"逐点用自己的时间戳预测"的语义；
//   · 量测矩阵变成 N×N，K 求解从 O(N) 退化为 O(N³)。
const float curv = msg->points[i].offset_time / float(1000000);
float cur_curv = curv;
if (has_prev_kept && std::fabs(curv - prev_kept_curv) >= 1.0f) {
    cur_curv = prev_kept_curv + 0.004166667f;  // 保持原兜底语义：时间跳变时前进一个点周期
}
pl_full[i].curvature = cur_curv;
prev_kept_curv = cur_curv;
has_prev_kept = true;
```

设计要点：

- **真实时间戳原样保留**（正常情况走 `cur_curv = curv`），只在**确实出现 ≥1 ms 跳变**时才用兜底值；
- 兜底语义与上游一致（前进一个点周期），仅把"参考点"从"上一个索引"改成"上一个被保留的点"；
- 两个状态变量都是**函数内局部变量**，每帧重置，无跨帧污染；
- 死代码 `if (i == 0)` 一并移除。

## 2.4 验证

### (a) 分组数与耗时（合成数据：静止房间，约 9600 点/帧）

| 指标                         | 修复前                      | 修复后                      |
| ---------------------------- | --------------------------- | --------------------------- |
| 每帧分组数（pf=3）           | **32**（最大组 3168） | **3199**（每组 1 点） |
| 每帧更新耗时 `icp`（pf=3） | **221 ms**            | **3.8 ms**            |
| 每帧更新耗时 `icp`（pf=4） | 19.6 ms                     | **1.9 ms**            |
| 耗时随点数关系               | 近似立方                    | **线性**              |

> 修复前 "点更少反而更慢"（pf=4 比 pf=3 快 11 倍）正是 O(N³) 的直接证据。

### (b) 精度（修复后，静止 / 匀速 0.3 m/s 各 10 s）

| 配置                             | 静止偏移    | 匀速 dy（真值 2.97 m） | 速度估计误差 | 每帧 `icp` |
| -------------------------------- | ----------- | ---------------------- | ------------ | ------------ |
| pf=3, iter=1                     | dx −1.2 mm | 2.9727 m               | +0.0002 m/s  | 7.2 ms       |
| **pf=3, iter=2（本项目）** | dx +4.0 mm  | 2.9780 m               | +0.0008 m/s  | 9.3 ms       |
| pf=4, iter=1                     | dx +0.5 mm  | 2.9692 m               | −0.0001 m/s | 3.9 ms       |
| pf=4, iter=2                     | dx −4.6 mm | 2.9696 m               | −0.0000 m/s | 5.4 ms       |

输出频率始终 10.00 Hz。

### (c) 被撤回的旧结论

修复前曾观察到"`pf=4` + 单次迭代在静止场景偏移 **0.20 m**"，并据此提出"两个参数要成对调整"。

修复后该现象**不再复现**（所有配置均毫米级）。原因是：上述偏差来自 **整帧塌缩成一组导致的批次线性化误差**，而非抽稀本身。**该结论作废**。

## 2.5 影响评估与风险

| 项                       | 说明                                                                                                                                                                                                            |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **正向**           | 恢复逐点运动补偿（Point-LIO 的核心机制）；CPU 大幅下降；odom 时间戳回到正确时刻                                                                                                                                 |
| **行为变化**       | 实车输出**会变**：帧内畸变补偿恢复后，原先被"整帧塌缩"掩盖的尺度/方向误差会重新暴露或修正                                                                                                                 |
| **历史标定需复核** | `mount_yaw=58°`、1.7% 尺度偏差都是在**bug 存在时**测的。建议重跑直行 1–3 m 与原地 180°。特别是尺度：修复前整帧被当作瞬时测量，推车 0.5 m/s × 0.1 s ≈ 5 cm 的系统性错位，可能是那 1.7% 的一部分来源 |
| **兼容性**         | 仅影响 `lidar_type == AVIA`（Livox 自定义格式）分支；velodyne / ouster / hesail / unilidar 分支未改动                                                                                                         |

---

# 3. iEKF 迭代次数可配置 + 协方差更新时机修正

```properties
[submodule "include/IKFoM"]
	path = include/IKFoM
	url = https://github.com/hku-mars/IKFoM.git
	branch = toolkit
```

**文件**：
[`include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp)、
[`src/parameters.cpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/parameters.cpp)、
[`src/parameters.h`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/parameters.h)、
[`src/laserMapping.cpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/laserMapping.cpp)

## 3.1 问题 A：迭代次数写死且无法从外部设置

`esekfom.hpp` 里 `maximum_iter`：

- 是 **private** 成员；
- 在 `init_dyn_share_modified()` 与 `init_dyn_share_modified_2h()` 里都被硬编码为 **1**；
- 全仓库没有其它赋值处（`rg "maximum_iter\s*="` 只有这两处 + 声明）。

结果："iterated EKF" 实际只做**一次**线性化（等价普通 EKF 更新），且用户无法调整。

## 3.2 修复 A：新增公开接口

```cpp
// esekfom.hpp:133-145
// 设置每次量测更新里 Gauss-Newton 的迭代次数。
// 必须在 init_dyn_share_modified*() 之后调用（那两处会把 maximum_iter 重置为 1）。
// 每次迭代都会重新计算 h(x)（含重新投影与近邻搜索），CPU 开销与迭代次数近似成正比。
void set_maximum_iter(int iters)
{
    maximum_iter = (iters > 0) ? iters : 1;
}

int get_maximum_iter() const
{
    return maximum_iter;
}
```

`init_*` **保持把 `maximum_iter` 重置为 1**，从而保证库的向后兼容——没有调用 setter 的调用方行为完全不变。

新增 ROS 参数：

```cpp
// parameters.h:26
extern int max_iter;

// parameters.cpp:50（默认 1 = 旧行为，保证其它 lidar 的 launch 不受影响）
nh->declare_parameter<int>("max_iter", 1);

// parameters.cpp:111
nh->get_parameter("max_iter", max_iter);
```

应用（必须在 init 之后）：

```cpp
// laserMapping.cpp:750-756
kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
// 必须在 init_* 之后设置：init 里会把 maximum_iter 重置为 1
kf_input.set_maximum_iter(max_iter);
kf_output.set_maximum_iter(max_iter);
RCLCPP_INFO(logger, "IEKF maximum_iter = %d", kf_output.get_maximum_iter());
```

## 3.3 问题 B：协方差在每次迭代里都被收缩

原实现把 `P` 的更新写在迭代循环**内部**：

```cpp
for (int i = 0; i < maximum_iter; i++) {
    ...
    K_ = PHT * HPHT.inverse();
    x_.boxplus(dx_);
    P_ = P_ - K_ * h_x * P_.block<12, n>(0, 0);   // ← 每轮都收缩
}
```

迭代 N 次的问题：

1. **同一个量测把 `P` 收缩了 N 次** → 协方差过度乐观（"我比实际更确定"）；
2. 下一轮的 `K` 基于**已收缩**的 `P`，偏离标准 Gauss-Newton（Gauss-Newton 应在**固定先验 `P`** 下迭代状态）。

## 3.4 修复 B：只在最后一轮更新协方差

```cpp
// esekfom.hpp:293-303（update_iterated_dyn_share_modified）
// 协方差只在最后一次迭代更新：
//  - 多次迭代时，若每轮都收缩 P，会让 P 在同一个量测上被收缩多次而过度乐观，
//    且下一轮的 K 会基于已收缩的 P，偏离标准 Gauss-Newton（应为固定先验 P）。
//  - maximum_iter == 1 时 i == 0 即最后一次，行为与原来完全一致。
if (i == maximum_iter - 1)
{
    P_ = P_ - K_*h_x*P_. template block<12, n>(0, 0);
}
```

```cpp
// esekfom.hpp:341-348（update_iterated_dyn_share_IMU，同样处理）
Matrix<scalar_type, n, 1> dx_ = K * z;

// 同 update_iterated_dyn_share_modified：协方差只在最后一次迭代更新
if (i == maximum_iter - 1)
{
    P_ -= K * HP;
}
x_.boxplus(dx_);
```

> **关键性质**：`maximum_iter == 1` 时 `i == 0 == maximum_iter - 1` 恒成立，两条路径与原实现**逐位等价**。因此这个修改对现有所有配置零风险。

## 3.5 验证

| 检查                                           | 结果                                                             |
| ---------------------------------------------- | ---------------------------------------------------------------- |
| `max_iter=1`（代码默认）                     | 启动日志 `IEKF maximum_iter = 1`，与旧版行为等价               |
| `max_iter=2`（mid360 launch）                | 启动日志 `IEKF maximum_iter = 2`                               |
| 其它 lidar launch（裸跑 `pointlio_mapping`） | `max_iter = 1`，**不受影响**                             |
| 精度收益（合成数据）                           | iter=1 vs 2 均为毫米级，**未观察到精度差异**；耗时 +20~40% |

> 因此 `max_iter=2` 在合成数据上属于"便宜的保险"：真实数据是非线性的（近邻会变、法向会变、有动态物体），第二轮重线性化有价值；但若 CPU 紧张，改回 1 是有依据的选择。

---

# 4. 启动自检日志

**文件**：[`src/laserMapping.cpp`](../point_lio/src/9000_point_lio_ros2_Mid-360/src/laserMapping.cpp) 717–720 行

```cpp
cout << "lidar_type: " << lidar_type << endl;
cout << "preprocess: scan_line=" << p_pre->N_SCANS
     << " point_filter_num=" << p_pre->point_filter_num
     << " blind=" << p_pre->blind
     << " max_iter=" << max_iter << endl;
```

实际输出：

```
lidar_type: 1
preprocess: scan_line=6 point_filter_num=3 blind=0.1 max_iter=2
[INFO] [laserMapping]: IEKF maximum_iter = 2
```

**为什么值得加**：`scan_line` / `blind` / `point_filter_num` 这三个参数一旦配错，**现场的日志里看不出任何异常**（表现为"精度莫名变差"）。这一行让每次启动都能自证配置，排查时不用再猜"到底加载了哪份参数"。

---

# 5. 参数调整

**文件**：[`launch/mapping_mid360.launch.py`](../point_lio/src/9000_point_lio_ros2_Mid-360/launch/mapping_mid360.launch.py) 26–27 行

```python
'point_filter_num': 3,  # Options: 1, 3, 4, 5（每 N 个点取 1 个，线性影响 CPU）
'max_iter': 2,          # 每次量测更新的 iEKF 迭代次数（1 = 单次 EKF；2 = 多一轮重线性化）
```

## 5.1 `point_filter_num` 为什么保持 3

曾在排查期间试过 4，最终回到 3，理由：

1. **CPU 已不是瓶颈**：修复时间戳 bug 后 pf=3 单帧仅 9.3 ms；真实 2 万点按比例放大约 20–35 ms（估算），对 100 ms 帧预算仍有 3 倍余量；
2. **5 近邻的硬约束**：残差依赖 **5 个最近邻**拟合平面，抽稀到 1/4 会让第 5 近邻可能落在 1 m 之外，法向精度下降；`plane_thr=0.1` 也更难通过；
3. **退化场景需要余量**：载荷遮挡、正对单面墙、走廊等场景有效点骤减，多的那 ~1.7k 点是"还能不能配上"的差别。

**换成 4 的条件**（任一满足即降）：

- `ros2 topic hz /aft_mapped_to_init` 低于 10 Hz；
- `top -H` 看 `pointlio_mapping` 单核 > 80% 且与 RViz/打印节点抢核；
- 机载算力很弱（低功耗 ARM 板），可进一步用 4,1。

## 5.2 `max_iter` 为什么取 2

- 合成数据上 2 与 1 精度无差异，但不增加架构复杂度、只在 LiDAR 更新时多一轮；
- 真实数据存在近邻/法向变化，第二轮重线性化是有效保险；
- 耗时增加 20–40%（9.3 ms vs 7.2 ms），可接受。

## 5.3 关于 `config/mid360.yaml`

该文件中的 `scan_line: 6`、`blind: 0.1`、`satu_acc: 78.5`、`satu_gyro: 34.9`、`acc_norm: 9.81`、`timestamp_unit: 1` 属于**此前的人工调参，本次未改动**。

顺带确认：`scan_line: 6` 对 Mid-360 是"全通过"——驱动里 `point.line = i % 4`（`kLineNumberMid360 = 4`），line 只取 0–3。因此这项既不筛点也不会丢点，**不是有效的调参旋钮**；想省算力请用 `point_filter_num`。

---

# 6. 验证汇总

## 6.1 构建

```bash
cd ~/point_lio && source /opt/ros/humble/setup.bash
colcon build --packages-select point_lio --symlink-install
# Finished <<< point_lio [7.64s]  
```

## 6.2 启动核对

```bash
source ~/livox_ws/install/setup.bash
source ~/point_lio/install/setup.bash
ros2 launch point_lio mapping_mid360.launch.py
```

```
lidar_type: 1
preprocess: scan_line=6 point_filter_num=3 blind=0.1 max_iter=2
[INFO] [laserMapping]: IEKF maximum_iter = 2
```

## 6.3 端到端（合成数据，直接使用真实 launch 文件）

为保证"改的是真实链路"而不是"另起一套参数"，测试用真实 `mapping_mid360.launch.py` 启动，另写一个合成 Mid-360 发布器（静止房间 + 可配置匀速运动，9600 点/帧、10 Hz、IMU 200 Hz）：

| 场景              | 结果                                                                   |
| ----------------- | ---------------------------------------------------------------------- |
| 静止 10 s         | 10.00 Hz，dx −0.8 mm / dz +0.1 mm，0 error                            |
| 匀速 0.3 m/s 10 s | 10.00 Hz，dy 2.9711 m（理论 2.97），v_est 0.3002，dx −3.4 mm，0 error |

> 合成数据是理想平面、无噪声、无运动畸变，只用于验证**功能正确性**与**量级**，不能替代实车标定。

## 6.4 仓库状态

```
 M include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp
 M launch/mapping_mid360.launch.py
 M src/laserMapping.cpp
 M src/parameters.cpp
 M src/parameters.h
 M src/preprocess.cpp
 M config/mid360.yaml          ← 此前人工调参，本次未动
```

无新增未跟踪文件；测试用的临时脚本已删除。

---

# 7. 回滚方式

```bash
cd ~/point_lio/src/9000_point_lio_ros2_Mid-360

# 全部回滚（含此前的人工调参）
git stash

# 或只回滚某个文件
git checkout -- src/preprocess.cpp

cd ~/point_lio && colcon build --packages-select point_lio --symlink-install
```

> `install/` 里 launch/config 是**符号链接**，但可执行文件是实体，**改 .cpp 必须重新 build**。

---

# 8. 待实车确认（重点）

| 优先级       | 检查项                   | 命令 / 方法                                                                       | 通过标准                                            |
| ------------ | ------------------------ | --------------------------------------------------------------------------------- | --------------------------------------------------- |
| **P0** | 时间戳修复是否真的生效   | `ros2 topic hz /aft_mapped_to_init`                                             | 稳定 **10.00 Hz**（修复前可能低于 10）        |
| **P0** | CPU 是否下降             | `top -H -p $(pgrep -f pointlio_mapping)`                                        | 单核 < 70%                                          |
| **P0** | odom 时间戳不再偏 100 ms | `ros2 topic echo /aft_mapped_to_init --field header.stamp` 与 `date +%s` 对比 | 差值 ≈ 帧时长（≈0.1 s），不是 0.004 s             |
| **P0** | **尺度是否改善**   | 直行 3 m（地面贴基准），读终点 Δy                                                | 与卷尺差 ≤ 1 cm（对应 ≤0.3%）；对比修复前 3.048 m |
| P1           | 横向误差                 | 同上，读 Δx                                                                      | ≤ 2 cm                                             |
| P1           | 静止抖动                 | 静止 10 s 读 `/global/chassis_odom`                                             | ≤ 5 mm                                             |
| P1           | 原地 180° 杠杆臂        | 原地转 180°                                                                      | 位移 ≈ 0.62 m（trim 后 0.66 m）                    |
| P1           | 盲区过滤                 | RViz 看 `/cloud_registered`                                                     | 自车周围无"贴脸"假平面                              |
| P2           | 长时间稳定性             | 连续跑 20 min                                                                     | 无掉帧、无内存持续增长                              |

## 8.1 如果实车结果变差，按这个顺序排查

1. **先看频率**：`ros2 topic hz` 若不是 10 Hz，说明 CPU 不足 → `max_iter` 改 1，再不行 `point_filter_num` 改 4；
2. **再看时间戳**：用 `ros2 topic echo --field header.stamp` 对比墙钟，若偏差不是 ≈100 ms，说明 `offset_time` 字段本身有问题（驱动侧）；
3. **再看尺度**：若 3 m 走成 3.05 m，且**修复前后一样**，说明与本次改动无关，是 `time_lag_imu_to_lidar` / `acc_norm` 层面的问题（见 `to_update.md` 3.1.2）；
4. **最后看外参**：若 Δx 系统性偏大且重复性好，才是 `mount_*` 的问题。

> **不要一上来就调 `mount_*`**。本次修复改变了帧内运动补偿，历史标定值可能对不上，但方向性误差（Δx）与尺度误差（Δy）要分开看。

---

# 9. 附：改动清单速查

```
src/preprocess.cpp
  + #include <cmath>
  avia_handler():
    + prev_kept_curv / has_prev_kept 局部状态
    ~ curvature 改为以"上一个保留点"为参考（保留真实时间戳）
    - 删除死代码 if (i == 0)
    ~ blind 判据：A||B||(C&&D) → (moved && range2 > blind²)
    ~ abs() → std::fabs()

include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp
  + set_maximum_iter(int) / get_maximum_iter()
  ~ update_iterated_dyn_share_modified(): P 仅在最后一轮更新
  ~ update_iterated_dyn_share_IMU():        P 仅在最后一轮更新

src/parameters.h    + extern int max_iter
src/parameters.cpp  + int max_iter; declare_parameter<int>("max_iter", 1); get_parameter
src/laserMapping.cpp
  + 启动打印 preprocess 参数行
  + init_* 之后 set_maximum_iter(max_iter) ×2
  + RCLCPP_INFO("IEKF maximum_iter = %d", ...)

launch/mapping_mid360.launch.py
  ~ point_filter_num 注释补充选项说明
  + max_iter: 2
```
