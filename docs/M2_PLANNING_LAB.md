# M2 — MoveIt 2 Planning Lab (SIMULATION ONLY)

> 本里程碑的所有内容都**只在仿真中运行**：ros2_control 使用 `mock_components/GenericSystem`，没有任何 Dobot 硬件插件。规划出的轨迹**不能也不会**发送到真实 CR10A。

## 1. 运行

```bash
source scripts/ex_env.sh
ros2 launch ex_moveit_config sim_moveit.launch.py            # RViz MotionPlanning + HUD
# 另开一个终端：
ros2 run ex_planning_lab benchmark --ros-args -p trials:=5 -p out_dir:=data/reports/planning_lab/run01
python3 evaluation/plot_planning_lab.py data/reports/planning_lab/run01 --out assets/m2
ros2 run ex_planning_lab execute_demo --ros-args -p scene:=wall -p query:=sweep_60 -p out_dir:=data/reports/planning_lab/exec_wall
```
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

## 5. 结果

见 `docs/evidence/m2_planning_benchmark_summary.md` 和 `assets/m2/*.png`。分析写在第 6 节。

## 6. 分析与 Failure Analysis

（基准跑完后，根据实测数据填写）
