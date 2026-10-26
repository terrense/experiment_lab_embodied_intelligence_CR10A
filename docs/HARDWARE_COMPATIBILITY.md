# Hardware Compatibility — Dobot CR10A × EMBODIED-X

> 状态标记：**VERIFIED**（有可复现证据）· **EVIDENCE**（有文档或静态分析支持，尚未在真机上验证）· **UNVERIFIED**（目前没有证据）· **RISK**
> 最后更新：2026-10-26（晚）。**已进行首次现场只读连接**（§7）。未特别标注的结论仍来自源码、官方文档、官方 SDK 和离线实验。

## 0. 结论摘要

| # | 问题 | 结论 | 状态 |
|---|---|---|---|
| 1 | CR10A 属于哪个产品族、该用哪个官方仓库 | CRA 系列，对应 V4 协议 → `DOBOT_6Axis_ROS2_V4`（官方描述原文是 "Demo of CRA/E6"） | EVIDENCE |
| 2 | 官方 `cr10` URDF 的几何能否用于 CR10A | **可以。** 真机 16055 个样本：URDF FK 与控制器 TCP 的误差 RMS 1.61 mm、max 2.85 mm；CR10AF 模型为 43.2 mm（§7） | **VERIFIED（真机）**，J5/J6 的运动覆盖尚不足 |
| 3 | 能否把 CR10、CR10AF、CR10A 当成同一款机器人 | **不能**。CR10 与 CR10AF 的 URDF 在相同关节角下法兰点恒差 **41.9 mm**，J3 限位也不同 | VERIFIED（针对 URDF 文件） |
| 4 | 关节限位 | CR10A 规格：J1/2/4/5/6 ±360°，J3 ±160°（零售商数据表）；V4 仓库的 cr10 URDF 写的是 J3 ±163.9° → 我们改用 ±160° 覆盖 | EVIDENCE |
| 5 | 关节命名和顺序 | `QActual[0..5]` → joint1..6；在 J2/J3/J4 大范围运动下 FK 残差始终 < 3 mm（任何映射或符号错误都会产生厘米到分米级误差） | **VERIFIED（真机）**，J1/J5/J6 只覆盖了静态位姿 |
| 6 | 单位与约定 | `QActual` 为度；`ToolVectorActual` 为 mm + 度，rx/ry/rz 为**外旋 XYZ**（R = Rz·Ry·Rx，与 URDF rpy 相同）；`ActualQuaternion` 顺序为 **(w, x, y, z)**；两者与 FK 的姿态差均为 0.42°，其他假设 ≥ 30° | **VERIFIED（真机）** |
| 7 | 反馈协议 1440 B 布局 | C++ 头文件、协议 PDF V4.6.5、官方 Python SDK 三方一致；37 个字段 0 差异 | VERIFIED（与官方资料一致；未经真机抓包） |
| 8 | 控制器固件版本是否兼容 V4 | 未知：需要在示教器或 DobotStudio Pro 里读出控制器版本（文档对应 V4.x） | UNVERIFIED |
| 9 | Link6 ↔ 控制器"法兰 / 工具 0"坐标系 | 恒等变换（tool 0 时位置差 1.6 mm、姿态差 0.42°） | **VERIFIED（真机）** |
| 10 | base_link ↔ 控制器基坐标系（user 0） | 重合（同上，FK 在 base_link 下与 user 0 的 TCP 吻合） | **VERIFIED（真机）** |

## 1. 型号归属证据

1. 越疆 GitHub 账号 `Dobot-Arm` 的仓库描述（2026-10-26 通过 API 抓取）：
   - `DOBOT_6Axis_ROS2_V4` — "About Demo of **CRA/E6**; Dobot V4 version ROS2"
   - `DOBOT_6Axis_ROS2_V3` — "About Demo of **CR/Nova**; Dobot V3 version ROS2"
   - `TCP-IP-Python-V4` — "**CRA/E6**; V4 version Api…"
2. V4 仓库里的描述包名为 `cra_description`。协议 PDF V4.6.5 中多处出现"CRA 机型""CRAF 机型（仅适用）"的字样。
3. 推论：CR10A 是 CRA 系列，用 V4 协议和 V4 仓库。仓库中的 `cr10` 很可能指 CRA 系列的 10 kg 机型（CR10A），`cr10af` 是 CRAF 变体。**这是推论，不是官方明文**。官方 README 的型号列表里只写了 `cr10`、`cr10af`，没有写 `cr10a`。

## 2. 运动学模型证据

### 2.1 三个官方仓库的 cr10 URDF 对比（joint origin / rpy / axis / limit）

| joint | origin xyz (m) | rpy | V3 (CR) 限位 | V4 (CRA) 限位 | ROS-6AXis 限位 |
|---|---|---|---|---|---|
| joint1 | 0 0 0.1765 | 0 0 0 | ±3.14 | ±6.27 | ±6.28 |
| joint2 | 0 0 0 | 1.5708 1.5708 0 | ±3.14 | ±6.27 | ±6.28 |
| joint3 | -0.607 0 0 | 0 0 0 | ±2.861 | ±2.861 | **±2.793** |
| joint4 | -0.568 0 0.191 | 0 0 -1.5708 | ±3.14 | ±6.27 | ±6.28 |
| joint5 | 0 -0.125 0 | 1.5708 0 0 | ±3.14 | ±6.27 | ±6.28 |
| joint6 | 0 0.1084 0 | -1.5708 0 0 | ±6.28 | ±6.27 | ±6.28 |

来源 commit：V4 `ec201c40`，V3 `34a9fcea`，TCP-IP-ROS-6AXis `0924e785`。所有轴都是 `0 0 1`。

**几何完全一致**，只有限位不同。由此得到等效 DH 量级：d1 = 176.5 mm，a2 = 607 mm，a3 = 568 mm，d4 = 191 mm，d5 = 125 mm，d6 = 108.4 mm（单元测试 `test_cr10_link_lengths` 已覆盖）。

### 2.2 CR10 与 CR10AF（同为 V4 仓库）对比

由 `scripts/compare_dobot_models.py` 生成 → `data/reports/model_compare_cr10_vs_cr10af.json`：

| 指标 | cr10 | cr10af |
|---|---|---|
| J3 限位 | ±163.92° | ±160.0° |
| 零位法兰 xyz (mm) | (0, −299.4, 1476.5) | (0, −257.5, 1476.5) |
| 相同 q 下法兰位置差（500 组随机 q） | mean 41.90 mm / max 41.91 mm | |
| 采样最大到达距离 | 1499.4 mm | 1497.1 mm |

解读：位置差是**常数**（mean ≈ max），说明两条运动链的关节轴和零位约定相同，只是"法兰点"在第 6 连杆上的位置不同（d6 分别是 108.4 和 66.5 mm）。因此 **CR10 ≠ CR10AF**（至少 URDF 对法兰的定义不同）。CR10A 的法兰到底对应哪个定义，看一次现场录制就能定：两个候选模型在 TCP 误差上会差出约 42 mm。

### 2.3 公开规格（二手来源，按 EVIDENCE 处理）

零售商数据表（RobotShop、Scan 等转载的越疆规格）：CR10A 工作半径 1300 mm，负载 10 kg，J1/2/4/5/6 ±360°，J3 ±160°，关节最大速度 J1/J2 150°/s、J3 180°/s、J4–J6 223°/s，重复定位精度 ±0.03 mm。
→ 关节限位覆盖层见 `scripts/vendor_dobot_cr10.py`；跳变检测使用的 `max_joint_velocity_deg_s: 225` 取自 223°/s。
官方 URDF 中的 `velocity="100"`（rad/s）明显是占位值，**不能**用于跳变检测。

### 2.4 理论 DH 与标定 DH

官方 `dobot_moveit/scripts/verify_dh.py` 明确说明：控制器内部使用**出厂标定 DH**，URDF/KDL 用的是**理论 DH**，两者之间存在偏差。所以即便模型正确，URDF FK 与控制器 TCP 之间也应该有毫米级残差。可视化可以接受这个量级；**如果要做精度控制，必须使用控制器 IK**。

## 3. 反馈协议

| 项 | 内容 | 证据 |
|---|---|---|
| 端口 | 29999 为 Dashboard 命令端口；30004（8 ms）推送实时数据，官方 README 和驱动只用这两个端口。协议 PDF §1 另写有 30005（200 ms）和 30006（可配置，默认 1000 ms），**本机是否开放未验证** | README + 协议 PDF §1 |
| 包长 | 1440 B，小端，`MessageSize=1440`，`TestValue=0x0123456789ABCDEF` | PDF + `command.h` |
| 偏移 | 本项目使用的 37 个字段与 `command.h` 注释一致（单元测试 `test_offsets_match_official_header`），也与官方 Python SDK `dobot_sdk/protocol/data_types.py` 的 `MyType` dtype 一致（itemsize 1440，0 差异，TCP-IP-Python-V4 @ `55ec1ec8`） | 自动化测试 + 一次性脚本 |
| QActual 单位 | 度 | 官方驱动 `deg2Rad(q_actual)` |
| QDActual 单位 | 推测为 deg/s | UNVERIFIED |
| ActualQuaternion 顺序 | 未文档化；交叉校验会同时评估 wxyz 和 xyzw 两种假设 | UNVERIFIED |
| TimeStamp | 控制器 Unix 时间（ms），与主机**不同步**；observer 统计偏移的中位数和标准差 | PDF；需现场测量 |

## 4. 官方驱动兼容性问题（静态审计，详见 SAFETY.md §3）

1. `dobot_bringup_v4` 发布的是 `/joint_states`；`dobot_rviz live_hardware:=true` 的 relay 订阅的却是 `/joint_states_robot`，而整个仓库中**没有任何节点发布 `/joint_states_robot`** → README 给出的两条命令组合起来，RViz 很可能只显示 relay 回退的**零位**（静态推断，未运行验证）。
2. bringup 断连时仍以当前时间戳发布最后一帧；从未连上时发布全零 → 无法区分"零位"和"无数据"。
3. relay 超过 1 s 没有数据时发布全零，并且按下标而不是按名字拷贝关节值。
4. 读取以固定 1440 B 分块，一次短读之后会永久错帧（每帧都 `len != 1440 → continue`）。

## 5. 未知问题与现场需完成的核验

见 `docs/LIVE_ROBOT_CHECKLIST.md`。关键项：
- [ ] 铭牌型号、序列号；控制器固件版本（≥ V4.x？）
- [ ] 30004 端口在 DobotStudio Pro 同时连接时能否再接受一个客户端；半关闭写方向（FIN）后是否持续推送
- [ ] `QActual` 与示教器显示的六个角度逐一对照（顺序、符号、单位）
- [ ] 一次 ≥ 60 s 的 LIVE_OBSERVE 录制（覆盖较大关节运动），离线运行 `evaluation/crosscheck_bag.py` → 判定法兰定义、四元数顺序和欧拉约定

## 6. 风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| 用错模型（CR10AF 法兰定义） | TCP 显示偏 42 mm；后续规划和手眼标定全部带系统误差 | 现场交叉校验之前，模型一律标注为 UNVERIFIED |
| 理论 DH 与标定 DH 的偏差 | 毫米级 TCP 误差 | 只用于可视化；控制必须用控制器 IK |
| 连接 30004 对控制器有副作用 | 未知（是否占用连接数、是否影响 DobotStudio） | 现场按清单逐步进行，先用 30005 低频端口 |
| 发错单位但数值小于 2π | 幅值检查拦不住（见 PROGRESS Failure Analysis） | 现场对照示教器读数，再加 FK 交叉校验 |

## 7. 现场只读连接结果（2026-10-26）

全程只读：只连接反馈端口，不连接 29999，不发送任何字节；观察期间由操作员手动拖拽（控制器 `robot_mode=6 BACKDRIVE`）。

### 7.1 网络
| 项 | 结果 |
|---|---|
| 物理连接 | 笔记本 USB 网卡（ASIX AX88179，100 Mbps）→ 现场局域网 |
| 地址 | 局域网内有 DHCP 服务器 **192.168.201.2**（同时下发默认网关），**控制柜 = 192.168.201.1**（不是 README 的 192.168.5.1）；笔记本获得 192.168.201.100/24 |
| WSL2 可达性 | NAT 模式下可以直接访问，ping 约 1.2 ms，不需要改 Windows 路由 |
| 反馈端口 | **30004 ✅ 推送**，**30005 ✅ 推送**，30006 ❌（3 s 内无数据）；192.168.201.2 上没有任何反馈端口 |
| TCP 半关闭（FIN） | ❌ **控制器收到 FIN 后立即断开连接** → `half_close_write` 改为 false（默认值已修改）。只读保护还剩三层：端口白名单、无发送 API、静态扫描 |
| 注意 | DHCP 下发了默认网关 192.168.201.2：Windows 可能把部分外网流量路由到现场网络。如果所属单位有要求，可以把该网卡的接口跃点数调高，或改用不带网关的静态 IP（`scripts/windows_robot_net.ps1 -Apply`，需要管理员权限） |

### 7.2 控制器状态与时钟
- 初次读取：`robot_mode=4 DISABLED`，tool 0 / user 0，负载 0.20 kg，无报警。之后操作员进入拖拽模式（6）。
- 控制器时钟**比主机慢 267.87 s**（中位数），接收抖动 σ = 6.9 ms（30004）。所有时间对齐必须使用主机接收时间，或显式估计这个偏移。
- 包周期：控制器时间戳步长中位数 8.0 ms（125 Hz）。

### 7.3 运动学模型判定（`evaluation/crosscheck_bag.py`）
| 录制 | 样本数 | 关节运动范围（J1..J6，°） | official_cr10 RMS / max | official_cr10af RMS |
|---|---|---|---|---|
| cr10a_live_01（拖拽 150 s） | 16055 | 9.9 / 57.0 / 20.5 / 73.7 / 0.24 / 0.00 | **1.61 / 2.85 mm**，姿态 0.42° | 43.16 mm，约 180° |
| cr10a_live_02（120 s，**机械臂未动**） | 14590 | 全为 0 | 1.60 mm，0.41° | 43.25 mm |
| 连续日志 crosscheck_20261026_204637.csv（整个会话 317 s，含一段 J1/J5/J6 大范围拖拽） | 38561 | **123.8 / 57.6 / 44.9 / 65.0 / 154.5 / 89.6** | **2.87 / 5.54 mm**（在线节点 n=41357：3.00 / 5.54 mm），姿态最优假设仍为外旋 XYZ | — |

结论：**CR10A 的运动学与官方 `cr10` URDF 一致**（Link6 = 法兰，base_link = user 0）。约 1.6 mm 的系统残差在所有位姿下几乎不变，符合理论 DH 与出厂标定 DH 的差异（`verify_dh.py` 也提到了这一点）。可视化可以接受这个量级；**精度控制必须使用控制器 IK**。

覆盖情况：两段 bag 录制中 J5/J6 几乎没动，但会话的**连续交叉校验日志**（kinematics_crosscheck 逐包写入的 CSV）完整记录了随后一段 J1 124°、J5 155°、J6 90° 的大范围拖拽，误差仍在 mm 量级（max 5.54 mm）。这样**六个关节的映射、符号和单位都得到了真机运动数据的约束**。

**残差随位姿变化**（起始位姿 1.60 mm，最终位姿 4.39 mm，不是常数）：这说明它来自几何参数差异（标定 DH 与理论 DH 的差），不是固定的安装偏移。下一步可以从这批数据中辨识 DH 修正量（运动学标定，列入 ROADMAP）；在此之前，精度控制依赖控制器 IK。

数据保存在本机 `data/live/`（真机数据，默认不公开）。

### 7.4 实测发现的软件问题
- observer 原先每次 `recv()` 只发布最新的一个包；TCP 经 NAT 会批量到达，导致约 **14% 的包被丢弃**（第一段录制中有 2140 个 > 12 ms 的间隔）。修复后（逐包发布），第二段录制中只有 4 个 > 12 ms 的间隔，p99 为 9 ms。
