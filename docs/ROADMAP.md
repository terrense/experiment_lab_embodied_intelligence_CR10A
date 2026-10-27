# Roadmap

原则：一次只推进一个 Milestone；每一步都要有可运行代码、测试、量化指标和 Failure Analysis。

## M1 — CR10A 只读数字孪生（当前）
- [x] SIMULATION / REPLAY / LIVE_OBSERVE（mock）三种模式，一键 launch
- [x] 校验 adapter、HUD、TCP 轨迹、TF 完整性、自动化验收、故障注入
- [x] 官方驱动审计；1440B 协议三方核验；CR10 / CR10AF 模型对比；离线模型辨识工具
- [x] **现场**：只读连接、RViz 实时同步、模型判定（官方 cr10 几何，1.6–5.5 mm；欧拉 = 外旋 XYZ，四元数 = wxyz）
- [ ] 运动学标定：用现场数据辨识 DH 修正量，把 FK 残差从约 3 mm 降下来（作为 M3 的前置）
- [ ] 真机与 RViz 并排录屏；量化端到端显示延迟（拍屏方式）
- [ ] Case Study #1：Read-only Digital Twin of a CR10A

## M2 — MoveIt 2 规划实验室（纯仿真）— 进行中
- [x] MoveIt 配置（mock 硬件）、3 个场景、400 次规划基准（run02）、clearance 区间指标、图表
- [ ] execute_demo：计划轨迹与执行轨迹的对比（mock，无动力学）
- [ ] BiTRRT 调参；时间预算扫描（0.5 / 2 / 5 s）；padding 对成功率的影响
- 真机执行前的硬约束：`default_robot_padding` ≥ 2 cm（M1 实测模型误差最大 5.5 mm，M2 路径余量中位数只有 1–2 cm）
- 用 M1 确定的 URDF 生成 MoveIt 配置（SRDF / 碰撞矩阵），使用虚拟 `JointTrajectoryController`
- Current / Target / Planned / Executed 四种状态同屏显示；轨迹跟踪误差分析
- OMPL 规划器基准（RRTConnect / RRT* / PRM / BIT*…）：成功率、时间、路径长度、clearance，同一障碍场景，多随机种子
- FK / IK / Jacobian / 时间参数化（TOTG vs Ruckig）讲解配合代码
- 真机执行：**不在 M2 范围内**

## M3 — T265 + 运动学轨迹对齐 + 手眼标定
- 先调研 T265 在 TX2 + 旧版 librealsense 上的情况（据我所知，T265 支持在 librealsense v2.54 前后被移除，**M3 开始前要核实**）；必要时由 TX2 采集，通过经过验证的接口传 pose
- AX = XB（Tsai / Park / Daniilidis），对比 FK 轨迹与 VIO 轨迹：ATE / RPE / 时间偏移估计
- 依赖 M1 的控制器时钟偏移统计

## M4 — 仿真器分工
- MuJoCo（本机优先）：从同一份 URDF 转换为 MJCF，验证动力学和控制器
- Gazebo（ros_gz，隔离环境）：系统级仿真
- Isaac Lab：远程 GPU；本机 RTX 5060 Laptop **8 GB**（不是 12 GB），只做轻量实验

## M5 — Active Perception / NBV（仿真）
Fixed Scan → Greedy NBV → Reachability-aware → Collision-constrained；指标包括覆盖率、视点数、路径长度、耗时、不可行率

## M6 — 学习与 Agent
PD/IK → 经典规划 → BC → PPO/SAC（必须有基线和消融实验）→ DR → Sim-to-Real 评测 → LLM 只输出结构化任务和受限技能调用
