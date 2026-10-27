# M2 — MoveIt 2 Planning Lab (SIMULATION ONLY)

> 本里程碑的所有内容都**只在仿真中运行**：ros2_control 使用 `mock_components/GenericSystem`，没有任何 Dobot 硬件插件。规划出的轨迹**不能也不会**发送到真实 CR10A。

## 1. 运行

```bash
source scripts/ex_env.sh
bash scripts/m2_session.sh start true      # 先清理残留实例，启动并确认只有 1 个 move_group（true = 开 RViz）
ros2 run ex_planning_lab benchmark --ros-args -p trials:=5 -p out_dir:=data/reports/planning_lab/run02
python3 evaluation/plot_planning_lab.py data/reports/planning_lab/run01 --out assets/m2
ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 -p out_dir:=data/reports/planning_lab/exec_wall
bash scripts/m2_session.sh stop
```
**一定要用 `m2_session.sh` 启停。** 残留的 move_group 会以相同的服务名继续服务，并且持有不同的规划场景，导致结果无法复现（见第 6 节）。benchmark 启动时如果发现 move_group 不止 1 个，会直接拒绝运行。
不要和 M1 的 `ex_bringup` 同时运行，否则两者都会发布 `/joint_states`。

## 2. 系统结构

```
RViz MotionPlanning ─┐                ┌─ /plan_kinematic_path (GetMotionPlan)   ← benchmark
ex_planning_lab ─────┼──► move_group ─┼─ /check_state_validity, /apply_planning_scene
                     │   (OMPL, KDL,  └─ /execute_trajectory (Action)            ← execute_demo
                     │    TOTG, PSM)
                     │        │ FollowJointTrajectory (Action)
                     │        ▼
                     │   arm_controller (JointTrajectoryController, 100 Hz)
                     │        │ position command          ▲ position/velocity state
                     │        ▼                           │
                     │   mock_components/GenericSystem ───┘ (理想位置跟随，无动力学)
                     │        │ joint_state_broadcaster → /joint_states → RSP → /tf
                     └──────── robot_state_monitor（HUD + 执行轨迹 /ex/tcp_path）
```

## 3. 关键概念（结合本项目代码）

**MoveGroup / PlanningScene。** move_group 是一个服务器节点：持有机器人模型（URDF + SRDF）和 PlanningScene（机器人当前状态 + 世界中的碰撞物体 + 允许碰撞矩阵 ACM），对外提供规划、校验和执行接口。benchmark 通过 `/apply_planning_scene` 加入盒子障碍（`apply_scene`），用 `/check_state_validity` 校验起止状态。和 ROS 1 MoveIt 相比，API 基本相同，但参数要通过 `MoveItConfigsBuilder` 注入各个节点，没有全局参数服务器。

**SRDF 与 ACM。** `ex_cr10.srdf` 定义规划组 `arm`（链 base_link → tcp）、命名状态和 `disable_collisions`。相邻连杆，以及"永远不会碰"的连杆对，都被排除在自碰撞检查之外（数据来自官方 cr10_moveit，MIT）。如果漏掉某个相邻连杆对，起始状态就会被判为"自碰撞"，所有规划都会失败。

**FK / IK / Jacobian。** FK 就是 `UrdfModel.fk`（`ex_robot_state/core/urdf_model.py`）：沿运动链连乘 `T_parent_joint(q)`。IK 求解 FK 的逆，MoveIt 用 KDL 数值 IK（`kinematics.yaml`，超时 0.05 s），它在 6 轴腕部奇异附近会变慢甚至失败。Jacobian J(q) = ∂x/∂q（6×6），它的奇异值决定了可操作度和速度传递关系，是 IK 迭代、笛卡尔速度控制和奇异性分析的基础。M1 的现场结果已经证明：**URDF FK 与控制器标定 FK 相差 1.6–5.5 mm**，所以精度要求高的目标点要使用控制器 IK（官方 `send_pose_target.py` 也是这么做的）。

**OMPL 与规划器类型。** OMPL 在关节空间做基于采样的规划，碰撞检测由 MoveIt 提供（FCL 对网格）。分为三类：
- 单查询双向树：RRTConnect、BiTRRT、SBL。通常最快，但不追求最优。
- 渐近最优：RRT\*、PRM\*。会**用满整个时间预算**来持续优化路径，所以它们的"规划时间"总是等于预算，不能拿来和其他规划器比速度。
- 基于投影 / 网格：KPIECE、BKPIECE、EST。依赖 `projection_evaluator: joints(joint1,joint2)`。
- 本机 MoveIt 2.5 的 OMPL 接口**没有注册 BITstar**（"Unknown planner"），已从配置中移除。

**时间参数化（TOTG）。** OMPL 输出的是没有时间信息的几何路径。请求适配器 `AddTimeOptimalParameterization` 在关节速度和加速度限制下给路径分配时间（`joint_limits.yaml` 使用 CR10A 规格速度，benchmark 用 0.3 倍缩放）。基准中的 "duration" 就是 TOTG 的输出。

**FollowJointTrajectory / JointTrajectoryController。** 执行时，move_group 通过 Action 把整条轨迹交给 `arm_controller`。控制器在 100 Hz 下插值并发送位置命令。Action 自带反馈、取消和目标容差检查，相当于 ROS 1 的 control_msgs action 接口，但由 ros2_control 的 controller_manager 负责加载。

**执行监控。** `execute_demo` 记录执行期间的 `/joint_states`，估计计划时间轴与执行时间轴之间的启动延迟（`estimate_time_offset`），再计算逐关节的 RMS 和最大跟踪误差。mock 硬件**没有动力学**，所以这个误差主要反映控制器的采样和插值，并不代表真机。真机会叠加伺服滞后、ServoJ 缓冲和网络延迟，这些留到后续用 MuJoCo 被控对象，以及在批准的条件下用真机验证。

## 4. 实验设计

- **场景**（`ex_planning_lab/scenes/*.yaml`，坐标系 world，盒子障碍；桌面上表面在 z = −0.01）
  - `wall`：机器人前方一面 0.9 m 高的薄墙，起止位姿对称地位于墙两侧 → 考察绕行能力
  - `window`：一面墙上开 0.5 × 0.4 m 的窗，需要从窗中穿过 → 狭窄通道
  - `shelf`：两层开放式货架，从下层移到上层 → 需要退出再进入的长绕行
- **查询**：显式关节查询 + 按 TCP 区域拒绝采样（固定种子，可复现），起止状态都经 `/check_state_validity` 校验
- **规划器**：RRTConnect、RRT、RRTstar、BiTRRT、PRM、PRMstar、KPIECE、BKPIECE、EST、SBL（OMPL 默认参数）
- **重复与预算**：每个规划器 × 查询重复 5 次，`allowed_planning_time = 2 s`，`num_planning_attempts = 1`，速度和加速度缩放 0.3

### 指标
| 指标 | 定义 |
|---|---|
| success | `error_code == SUCCESS` |
| planning_time | MoveIt 报告的规划时间（含时间参数化） |
| joint path | 关节空间折线长度 Σ‖Δq‖（rad） |
| TCP path | TCP 笛卡尔折线长度（m，用 FK 计算） |
| duration | TOTG 轨迹总时长（s） |
| **clearance 区间** | 沿路径（关节空间按 ≤ 0.02 rad 加密）的机器人与障碍物最小距离：**下界** = 胶囊模型（胶囊从网格拟合，必然包住网格）；**估计值** = 网格表面采样点（间距 5 mm，所以估计值 ≥ 真值，且 ≤ 真值 + 5 mm），只在胶囊距离最近的 15 个状态上计算 |

clearance 的局限：
- 估计值只在"胶囊距离最近"的状态上计算，这是一个启发式，但两种距离高度相关。
- 排除了 `base_link` 与 `table`、`Link1` 与 `table` 这两对：它们与桌面的距离不随关节角变化。
- MoveIt 的碰撞检查本身没有 padding，规划器会让路径紧贴障碍物，所以 clearance 中位数在几 mm 量级是正常的。

## 5. 结果（run02，2026-10-27，仿真数据）

400 次规划（3 个场景 × 8 个查询 × 10 个规划器 × 5 次），单一 move_group，用时 834 s。完整表格：`docs/evidence/m2_planning_benchmark_summary.md`。错误码只有两种：**成功 178、超时 222**。

![success](../assets/m2/success_rate.png)
![time](../assets/m2/planning_time.png)
![clearance](../assets/m2/clearance.png)
![window](../assets/m2/tcp_paths_window.png)

| 规划器 | 总成功率 | 规划时间中位数 wall / window / shelf (s) |
|---|---|---|
| **RRTConnect** | **39/40** | 0.044 / 0.619 / 0.230 |
| **SBL** | **39/40** | 0.731 / 0.970 / 1.156 |
| EST | 23/40 | 0.128 / 0.605 / 0.903 |
| BKPIECE | 23/40 | 0.942 / 1.760 / 1.722 |
| KPIECE | 20/40 | 0.124 / 1.001 / 0.700 |
| PRMstar | 14/40 | 2.03 / 2.13 / 2.12（时间预算用满） |
| PRM | 12/40 | 0.082 / — / 1.425 |
| RRTstar | 5/40 | — / — / 2.05（时间预算用满） |
| RRT | 3/40 | — / — / 0.533 |
| BiTRRT | 0/40 | — |

### 5.1 计划轨迹与执行轨迹对比（`execute_demo`，wall/sweep_60，RRTConnect，mock 控制器）
| 指标 | 值 |
|---|---|
| 计划时长 / 执行采样 | 5.58 s / 666 个样本（100 Hz） |
| 估计启动延迟（目标被接受 → 开始运动） | 24 ms |
| 关节跟踪 RMS（未对齐） | 0.13–1.11° |
| 关节跟踪 RMS（按延迟对齐后） | 0.01–0.08°（最大 0.24°） |
| 终点误差 | ≤ 0.055° |
| TCP 路径长度（计划 / 执行） | 2.093 / 2.094 m |

解读：在无动力学的理想被控对象上，"跟踪误差"几乎全部来自 24 ms 的时间轴偏移；对齐后剩下的只是控制器的插值和采样误差。这验证了分析管线可用，**不代表真机性能**。原始数据：`docs/evidence/m2_execute_wall_sweep60.json`。

踩坑：mock 机器人上电时处于 home 位姿，前臂正好穿过 wall 场景的墙 → 起始状态碰撞，所有规划都失败（"invalid states [0..3]"）。修复：定位运动只保留桌面，到位后再加入障碍物，并校验当前状态是否有效。

## 6. 分析与 Failure Analysis

### 6.1 规划器层面的发现（只在本实验设置下成立：关节空间目标、±1e-3 rad 容差、2 s 预算、OMPL 默认参数）
1. **双向树占优**：RRTConnect 和 SBL 都是 39/40。目标是一个极小的关节空间区域（6 维、±1e-3 rad）时，从目标侧也长一棵树几乎是必要的。单向树 RRT / RRTstar 只能靠 goal bias 去"撞"这个区域，在 2 s 内几乎全部超时（3/40、5/40）。这不能说明"RRT 差"，只能说明**单向树 + 点目标 + 短预算**这个组合差。
2. **狭窄通道把规划器区分开了**：window 场景中 PRM 为 0/15，PRM\* 为 1/15。均匀采样在 2 s 内很难把节点落进窗口通道；RRTConnect 和 SBL 仍有 14/15，但规划时间是 wall 场景的约 14 倍。
3. **渐近最优规划器总是用满预算**（2.0 s），路径改善却很有限：wall 场景 PRM\* 4.84 rad vs RRTConnect 5.29 rad；shelf 场景 RRT\* 9.76 rad vs RRTConnect 10.74 rad。按"时间换质量"衡量，在这个预算下不划算。
4. **BiTRRT 0/40**：在 MoveIt 默认参数下全部超时。原因**尚未查明**（T-RRT 的温度和代价设置可能并不适合"关节空间 + 无代价地图"的情形），留待调参实验，在此之前不下结论。

### 6.2 与 M1 真机结果的联系：clearance 预算
- 规划器会把路径贴着障碍物走：window/shelf 场景的 clearance 估计中位数只有 **1–2 cm**（RRTConnect 在 shelf 场景为 6 mm），而且它们满足"无碰撞"的前提只是**模型精确**。
- M1 现场测得 **URDF FK 与真机相差 1.6–5.5 mm**（理论 DH 与标定 DH 的差）。所以对真机而言，6 mm 的余量几乎被模型误差吃光了。**任何真机执行前，都必须设置 `default_robot_padding`（建议 ≥ 2 cm）**，并以真机的标定误差为依据。这条约束已写进 ROADMAP。
- wall 场景 clearance 恒为 0.109 m：最小值来自 Link2（肩部）与桌面之间，与路径基本无关。说明该指标需要区分"任务障碍"和"安装平面"，下一版会改成只对任务障碍报告。

### 6.3 实验方法本身的失败（run01 作废的原因）
| 问题 | 现象 | 根因 | 修复 |
|---|---|---|---|
| **多个 move_group 并存** | 同一状态的有效性时真时假；规划报 "goal state invalid"；window 场景里出现 shelf 场景的物体 | 之前运行留下的 MoveIt 实例没被杀掉（非交互 shell 的后台进程组 / 信号问题），多个实例以相同的服务名提供服务，并且持有不同的场景 | `scripts/m2_session.sh`（锚定匹配启停、`--no-daemon` 计数）；benchmark 启动时断言只有一个 move_group |
| **ros2 daemon 缓存** | 刚杀掉的节点仍出现在 `ros2 node list` 中 | daemon 的发现缓存 | 计数用 `--no-daemon`，stop 时同时停掉 daemon |
| **碰撞检查穿模** | 92 次 "Computed path is not valid"，OMPL 提示 "path may slightly touch invalid region" | 检查步长 = 0.005 × 状态空间最大跨度。CR10A ±360° 的限位使跨度达到 28.65 rad，相当于每 8.2° 才检查一次，足以穿过 4 cm 厚的墙 | `longest_valid_segment_fraction = 0.00035`（约 0.01 rad）。官方配置的 ±6.27 rad 限位有同样的问题 |
| 查询贴边 | 采样得到的状态可能紧贴障碍物 | 拒绝采样只校验了"无碰撞" | 要求 mesh clearance ≥ 3 cm（run02 中最小为 3.3 cm） |
| 胶囊过度保守 | clearance 全为负值 | 胶囊必须包住网格，Link2 的半径达到 0.15 m | 改为区间报告：胶囊下界 + 5 mm 表面采样估计 |

教训：**在比较规划器之前，先验证测量装置本身**（单一服务端、检查分辨率、查询余量）。run01 的"规划器排名"几乎完全是这些实验假象造成的。

### 6.3b 复测（run03，2026-10-27）：同一台机器上的计时漂移
怀疑 run02 期间残留的监视进程（见 M4 失败分析 #6）拖慢了规划，于是在干净环境下重跑了一遍。**结果和预期相反：run03 总耗时 2274 s，而 run02 是 834 s**；规划时间中位数普遍高出 1.5–3 倍，所以 2 s 预算内的成功率也更低（`docs/evidence/m2_planning_benchmark_run03_summary.md`）。M2 期间机器人是静止的，监视进程基本空闲，所以 run02 并没有被明显拖慢。真正的问题是**这台笔记本在不同时段的算力差异很大**：复测时电池电量 7%，电源和温度状态未知，**无法事后证实原因**。

结论：
- 跨时段的绝对规划时间不可比，只能比较同一轮内的结果。
- 两轮共同成立的结论保留：RRTConnect 与 SBL 最可靠；RRT、RRT*、BiTRRT 在点目标、2 s 预算下几乎失败；PRM 在狭窄通道中失败。
- 以后的计时实验要记录 CPU 和电源状态，并在同一轮内交替运行各个规划器。

### 6.4 局限
- mock 硬件没有动力学，执行跟踪误差（`execute_demo`）不代表真机。
- 每个查询只重复 5 次，成功率的置信区间较宽（例如 14/15 的 95% 区间约为 0.70–1.0），用来区分规划器之间的大差异足够，但不适合排列相近规划器的先后。
- OMPL 参数全部是默认值，没有调参；时间预算只有 2 s 一档。
