# Architecture — Milestone 1: Robot State → TF → RViz

## 1. 数据流（三种模式共用下游）

```
 Mode A  mock_joint_publisher ──┐  (SIMULATED, 125 Hz, 可注入故障)
 Mode B  ros2 bag play ─────────┤  (只回放 /joint_states_robot)
 Mode C  dobot_feedback_observer┤  (TCP 30004 只读 → 1440B 包 → 度转弧度)
                                ▼
                    /joint_states_robot   sensor_msgs/JointState  (BEST_EFFORT 订阅)
                                │
                    ┌───────────▼───────────┐
                    │  joint_state_adapter  │  名字映射 · 校验 · 新鲜度 · 冲突检测
                    └───────────┬───────────┘  ERROR → 不转发（冻结）
                                │ /joint_states (RELIABLE)        /diagnostics
                    ┌───────────▼───────────┐
                    │ robot_state_publisher │  URDF(xacro) + q → TF
                    └───────────┬───────────┘
                         /tf, /tf_static
            ┌───────────────────┼─────────────────────┐
            ▼                   ▼                     ▼
          RViz2      robot_state_monitor       pipeline_check (测试时)
                     HUD · TCP pose · 轨迹 · TF完整性 · /tf vs FK · JSONL 日志

 Mode C 附加：/ex/dobot/tool_vector_raw ─► kinematics_crosscheck (URDF FK vs 控制器 TCP)
                                        └► evaluation/crosscheck_bag.py (离线, 多候选模型)
```

## 2. 坐标系

```
world ─(mount, fixed)─► base_link ─joint1─► Link1 ─joint2─► Link2 ─joint3─► Link3
      ─joint4─► Link4 ─joint5─► Link5 ─joint6─► Link6 ─(fixed, 恒等 [待核验])─► flange ─(tool)─► tcp
```
- `world`：RViz 的 Fixed Frame，也是实验室坐标系。后续 T265 和手眼标定都挂在这里。
- `base_link` 对应控制器 user 0；`flange` 对应控制器 tool 0。两者都是**假设，需要现场核验**。
- URDF 中关节坐标系与子连杆坐标系重合，所以 HUD 上的关节轴箭头 = `R_world_child · axis`。

## 3. 包结构

| 包 | 类型 | 内容 |
|---|---|---|
| `ex_description` | ament_cmake | 由脚本从官方仓库生成的 CR10 宏和网格（MIT），加上 `ex_cr10.urdf.xacro`（world / flange / tcp） |
| `ex_robot_state` | ament_python | `core/`（**不依赖 ROS**：SE3、URDF/FK、校验、新鲜度、mock 运动、1440B 解析、只读 socket、交叉校验）+ `nodes/`（薄 ROS 封装）+ `tools/mock_dobot_controller` |
| `ex_bringup` | ament_cmake | `embodied_x.launch.py`、三种模式的 YAML、RViz 配置 |

设计取舍：核心逻辑与 rclpy 解耦，67 个单元测试不需要 ROS 也能运行；后续接 MuJoCo、Isaac 或离线评测时直接复用 `core/`。**没有**搭建通用的"机器人抽象层"，等出现第二个真实用例（比如 MuJoCo 状态源）再提炼接口。

## 4. 关键 ROS 2 概念（结合本项目代码；面向有 ROS 1 经验的读者）

### Node / Executor
- ROS 1 中一个进程对应一个节点，由 `ros::spin()` 驱动。ROS 2 中节点是对象，**Executor** 负责调度回调。我们的节点都用 `rclpy.spin(node)`，也就是 SingleThreadedExecutor，回调串行执行，不用加锁。
- 例外是 `dobot_feedback_observer`：阻塞式的 `recv()` 放在独立线程里，避免卡住 executor，它和诊断定时器共享 `last_pkt` 时加了锁（`_lock`）。
- 故障模式：在回调里做阻塞 IO，会导致定时器饿死、诊断停更。这正是 observer 使用独立线程的原因。

### Topic 与 QoS（ROS 1 中没有对应物）
- ROS 1 的 TCPROS 总是"可靠 + 队列"。ROS 2 基于 DDS，每个端点都要声明 QoS，**不兼容时不报错，只是收不到数据**。
- 本项目的选择（`nodes/common.py`）：
  - `/joint_states_robot` 用 **BEST_EFFORT** 订阅。它与可靠和尽力两种发布者都兼容，深度 5，只要最新值。
  - `/joint_states` 用 **RELIABLE** 发布。robot_state_publisher 以 RELIABLE 订阅，如果我们发布 BEST_EFFORT，RSP 会**静默收不到任何数据**。
  - `/robot_description` 是 **TRANSIENT_LOCAL**，相当于 ROS 1 的 latched topic，后启动的节点也能拿到 URDF（`load_model()` 就是这样回退获取的）。
- 调试：`ros2 topic info -v /joint_states` 可以看两端的 QoS（集成测试会保存到 `sim_topic_info_joint_states.txt`）。

### TF2
- 和 ROS 1 的 tf2 基本一致；静态变换走 `/tf_static`（TRANSIENT_LOCAL）。robot_state_publisher 发布的 TF 时间戳 = JointState 的 `header.stamp`。
- 所以 `pipeline_check` 和 `robot_state_monitor` 用 **JointState 的时间戳**去查 TF（`lookup_transform(world, tcp, stamp)`），再与独立计算的 FK 比较。用"最新"时间去查，运动中会有一帧的时间差，引入假误差。实测（run06）：/tf 与 FK 的最大误差为 0.0 mm（保留 6 位小数）。
- 两个来源同时发布同一个 TF 会导致显示来回跳。monitor 会检查 `/tf` 的发布者是否只有 `robot_state_publisher`。

### Service / Action
- 本里程碑**刻意不提供任何 Service 或 Action**。对照官方 bringup：约 130 个无鉴权的 Service 可以直接驱动机器人（SAFETY §3）。
- Action（例如后续 MoveIt 的 `FollowJointTrajectory`）= 带反馈和取消的长任务，相当于 ROS 1 的 actionlib；它会在 M2 的虚拟控制器里出现。

### 时间戳语义
- `stamp_source: receive`（默认）：adapter 用主机接收时刻重新打时间戳。REPLAY 必须这样做，因为 bag 里是过去的时间，TF 会被当成过期。
- 控制器的 `TimeStamp`（ms）是控制器时钟，**和主机不同步**。observer 统计 `host_wall − controller_ms` 的中位数和标准差（clock offset 与抖动），用于后续 VIO 和手眼标定的时间对齐分析。
- 端到端显示延迟目前只测到"接收到 /joint_states 发出"这一段（`stamp_latency_ms`，同一主机时钟）；RViz 渲染延迟需要通过拍屏方式测量（列入 ROADMAP）。

## 5. 可观测性
- `/diagnostics`：`ex/joint_state_adapter`、`ex/tf_integrity`、`ex/dobot_feedback_observer`、`ex/kinematics_crosscheck`
- RViz HUD（`/ex/hud`），颜色分级：绿色 VALID、黄色 WARN、红色 INVALID
- 健康日志：`~/.ros/embodied_x/<mode>_<ts>.jsonl`，每秒一行
- 测试报告：`data/reports/<run>/`（JSON + 日志 + bag 信息）
