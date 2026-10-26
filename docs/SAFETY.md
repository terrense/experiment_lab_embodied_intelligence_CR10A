# Safety Architecture

## 1. 原则

1. **未经现场明确授权，不运行任何连接真机的代码。** 仓库里所有默认配置、脚本和 CI 都不会连接 `127.0.0.1` 以外的地址。
2. **Milestone 1 只允许单向数据流：Physical Robot → ROS 2 → RViz。** RViz、MoveIt 和本项目的任何节点都不能反向作用于真机。
3. **"不调用 MoveIt"不等于只读。** 只读必须体现在**网络层**：不打开命令端口，也不发送任何字节。
4. **数据无效时宁可冻结并标红，也不显示"看起来正常"的假数据。**
5. 运动控制模块即使以后实现，也默认禁用，并且必须单独经过安全审核、控制器适配和人工确认。

## 2. 三种运行模式及其隔离

| 模式 | 数据源 | 网络连接 | 如何启用 |
|---|---|---|---|
| A SIMULATION | `mock_joint_publisher`（数据标注为 SIMULATED） | 无 | 默认 |
| B REPLAY | `ros2 bag play --topics /joint_states_robot` | 无 | `mode:=replay bag:=...` |
| C LIVE_OBSERVE | `dobot_feedback_observer` | 只连 30004/30005/30006 中的一个端口，只接收数据 | **两道门**：`live_observe.yaml: enable_live_connection: true` **且** `live_ack:=I_AM_ON_SITE_AND_AUTHORIZED` |

三种模式下数据源之后的链路完全相同（adapter → RSP → TF → RViz），所以 A 和 B 中验证过的东西在 C 中依然成立。

REPLAY **只回放原始源话题**。如果同时回放 `/joint_states` 或 `/tf`，就会和在线的 adapter / RSP 冲突，形成两个发布者，TF 会来回跳。adapter 每 0.5 s 检测一次 `/joint_states` 的发布者数量，一旦多于 1 个就报 ERROR。

## 3. 官方驱动 `dobot_bringup_v4` 审计（静态，commit `ec201c40`）

| 发现 | 位置 | 对只读的影响 |
|---|---|---|
| 启动后同时连接 **30004（反馈）和 29999（Dashboard 命令）**，并在后台线程中断线重连 | `src/command.cpp` `CRCommanderRos2` 构造函数及 `recvTask()` | 控制通道常开 |
| 注册约 **130 个 Service**，直接把命令写到 29999，包括 `EnableRobot`、`PowerOn`、`ClearError`、`SpeedFactor`、`SetPayload`、`SetTool`/`Tool`/`User`、`MovJ`/`MovL`/`ServoJ`/`ServoP`/`MoveJog`、`RunScript`、`StartDrag`、`BrakeControl`、`SetSafeSkin`、`SetCollisionLevel`、`RequestControl`… | `src/cr_robot_ros2.cpp` `init()` | 同一 ROS domain 内的**任何进程**都能调用，没有鉴权 |
| 启动时**不会**主动发送命令（`init()` 里没有 `doTcpCmd`） | 同上 | 启动本身没有命令副作用；但"打开 29999 连接"本身是否有副作用，例如抢占控制权或占用连接数，**未知** |
| 断连时以当前时间戳继续发布最后一帧；从未连上时发布**全零** | `src/main.cpp` | 下游无法区分零位和无数据 |
| 固定 1440 B 读取；一次短读后永久错帧 | `command.cpp recvTask()` + `tcp_socket.cpp` | 反馈可能静默中断 |
| `real_time_data_` 被接收线程写入，同时被发布线程无锁读取 | `cr_robot_ros2.cpp pubFeedBackInfo()` | 数据竞争（显示偶尔撕裂） |
| relay（`dobot_rviz`）：订阅的话题没有发布者；超过 1 s 发布全零；按下标拷贝关节值 | `dobot_rviz/joint_state_relay.py` | 见 HARDWARE_COMPATIBILITY §4 |

**结论：LIVE_OBSERVE 不使用官方 bringup。** 改用自研的 feedback-only observer。官方代码只用于参考包格式和模型几何。

## 4. 本项目的只读实现（每一条都有自动化测试）

| 机制 | 实现 | 测试 |
|---|---|---|
| 端口白名单：只允许 {30004, 30005, 30006}，拒绝 29999、30003 及其他端口 | `core/readonly_socket.py` | `test_command_and_other_ports_refused` |
| 不暴露发送 API；原始 socket 是私有成员 | 同上 | `test_no_raw_socket_attribute_exposed` |
| 静态扫描：live 路径的源码中不允许出现 `.send/.sendall/.sendto/.write/.makefile(` | — | `test_no_send_calls_in_live_path` |
| TCP 半关闭 `shutdown(SHUT_WR)`：内核层面禁止写 | `ReadOnlyFeedbackSocket.connect` | `test_end_to_end_..._sends_zero_bytes[True]`（写操作抛出 OSError） |
| 端到端验证：mock 控制器统计客户端发来的字节数，必须为 0 | `tools/mock_dobot_controller.py` | 单元测试 + 集成测试 stage 4（run06：2785 个包，**0 字节**） |
| 两道使能门；默认 `false` | launch + 节点内部双重检查 | 集成测试使用 127.0.0.1 |
| 集成脚本拒绝非回环地址 | `scripts/integration_test.sh` | — |
| 节点不提供任何 Service 或 Action | `dobot_feedback_observer.py` | 代码审查 |

**仍有残余风险（UNVERIFIED）**：真实控制器收到 FIN（半关闭）后是否会断开连接；30004 的连接数上限；与 DobotStudio Pro 同时连接时的行为。请按 LIVE_ROBOT_CHECKLIST §2 逐项确认。如果控制器在收到 FIN 后断开，把 `half_close_write: false`。此时仍有白名单、无发送 API 和静态扫描三层保护。

## 5. 数据有效性管线（adapter）

- 结构检查：长度、重名、缺失关节（ERROR），多余关节（WARN）
- 数值检查：NaN/Inf、`|q| > 2π+0.2`（疑似单位为度）、超限位（ERROR）、接近限位（WARN）
- 时间检查：跳变（`|Δq| > v_max·dt·1.5`，v_max = 225°/s，取自 CR10A 规格）、时间戳回退或为零、断连 > 1 s 后的 RESYNC
- 新鲜度：FRESH / STALE（> 100 ms）/ LOST（> 1 s）
- **ERROR 时不转发**：RSP 不会收到新数据，TF 和 RViz 冻结在最后一个有效帧，HUD 变红显示 INVALID 和原因（见 `assets/screenshots/m1_fault_degrees_rviz.png`）
- 诊断窗口内锁存最坏级别，单帧故障（8 ms 跳变）也不会被下一帧覆盖

**已知局限**：发错单位的值如果小于 2π，就会通过幅值检查（例如 −6.06° 被当成 −6.06 rad）。只能靠现场比对示教器读数和 FK 交叉校验来发现。

## 6. 网络与 DDS

- `ROS_LOCALHOST_ONLY=1`，`ROS_DOMAIN_ID=42`（`scripts/ex_env.sh`）：DDS 流量不进入实验室网络，避免别人的 ROS 节点调用我们，也避免我们调用别人。
- 连接真机时，只需要 TCP 到控制器 IP 的 30004 端口，**不需要 DDS 出网**。
