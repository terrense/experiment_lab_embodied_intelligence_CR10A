# PROGRESS

## 2026-10-26 — Milestone 1, session 1: offline pipeline + read-only live path (no hardware)

### Step 0 — 环境调查（实测结果）

| 项 | 结果 |
|---|---|
| OS | Windows 11 Home China 10.0.26200；WSL 2.7.10，内核 6.18.33.2，WSLg 1.0.73 |
| WSL 发行版 | `Ubuntu-22.04`、`Ubuntu-24.04`（属于其他项目，**未改动**）；docker-desktop |
| 新建 | **`EmbodiedX-Humble`**（Ubuntu 22.04.5，位于 `C:\WSL\EmbodiedX-Humble`，用户 `ex`），由 `scripts/setup_wsl_humble.sh` 配置 |
| ROS 2 | 原先没有 → 新发行版中装好 **Humble desktop**（rviz2、rsp、rosbag2、xacro、colcon），Python 3.10.12，numpy 1.21.5，pytest 6.2.5 |
| GPU | **NVIDIA GeForce RTX 5060 Laptop GPU，8 GB**，驱动 572.97。⚠️ 与你描述的 RTX 5070 12 GB 不一致，ROADMAP 已按 8 GB 规划 |
| Docker | 29.6.1（本次未使用） |
| 网络 | Windows 侧设有代理 127.0.0.1:7890（Clash TUN / fake-ip，DNS 解析到 198.18.x）。**WSL NAT 访问外网间歇性失败**，GitHub 的 git pack 传输极慢，codeload tarball 很快 |
| Git | 2.55；`embodied-x` 新建了仓库 |
| 真机 | 未连接，也没有尝试连接 |

### 已实现
- ROS 2 包：`ex_description`、`ex_robot_state`、`ex_bringup`（见 ARCHITECTURE.md）
- 节点：`mock_joint_publisher`、`joint_state_adapter`、`robot_state_monitor`、`dobot_feedback_observer`（只读）、`kinematics_crosscheck`、`pipeline_check`；工具 `mock_dobot_controller`
- 一键 launch，三种模式；两道使能门；RViz 配置（模型、TF、HUD、关节轴、TCP 位姿和轨迹、控制器 TCP）
- 脚本：环境配置、third_party 拉取（按 commit 锁定）、vendoring（含 CR10A 限位覆盖）、构建、单元测试、4 阶段集成测试、录制、截图、模型对比
- 离线模型辨识：`evaluation/crosscheck_bag.py`
- 文档：ARCHITECTURE / HARDWARE_COMPATIBILITY / SAFETY / LIVE_ROBOT_CHECKLIST / ROADMAP

### 实际执行的命令（节选）
```bash
wsl --install Ubuntu-22.04 --name EmbodiedX-Humble --location C:\WSL\EmbodiedX-Humble --no-launch --web-download
# apt 在线安装因 WSL 网络中断 → 离线：
apt-get install --print-uris ... > uris.txt     # WSL 内，1077 个包 / 556 MB
curl (Windows 侧, 8 并发) -> debs/ ; cp debs/*.deb /var/cache/apt/archives/ ; apt-get install --no-download ...
curl codeload.github.com/.../tar.gz/<sha>       # 官方仓库 main + v4-optimization
python3 scripts/vendor_dobot_cr10.py
bash scripts/build.sh ; bash scripts/test.sh ; RUN=run06 bash scripts/integration_test.sh
python3 scripts/compare_dobot_models.py --out data/reports/model_compare_cr10_vs_cr10af.json
powershell -File scripts/capture_rviz_window.ps1 -Out assets/screenshots/...
```

### 测试结果（真实运行）
- **单元测试 67/67 通过**（`docs/evidence/m1_unit_tests.xml`）
- **集成测试 run06：ALL STAGES OK**（`docs/evidence/m1_integration_run06_summary.txt`）

| 阶段 | 关键指标 |
|---|---|
| SIMULATION（25 s） | 125.38 Hz，max gap 11.28 ms，p99 10.23 ms；9 个 frame 全部存在；/tf 与 FK 的误差 0.0 mm；频率辨识映射 = [0..5]（正确）；单一发布者 |
| rosbag2 录制 | 21.4 s，6739 条消息 |
| REPLAY | 125.62 Hz，各项检查与仿真一致（映射、FK 均通过） |
| fault=degrees | adapter ERROR；拒收样本，显示冻结（max gap 5.01 s） |
| fault=dropout | LOST → ERROR（max gap 2.01 s） |
| fault=jump | 单帧 +20° 被 JUMP 检出（max gap 47 ms = 被拒的几帧） |
| LIVE_OBSERVE（mock 控制器，127.0.0.1） | 112.9 Hz；**控制器发出 2785 个包，收到 observer 发来 0 字节**；随机分块和注入的垃圾字节都能重新同步 |
| 离线模型辨识（mock） | official_cr10 rms **0.0 mm** vs official_cr10af **41.898 mm**，能选出生成数据的模型 |

截图：`assets/screenshots/m1_sim_rviz.png`、`m1_fault_degrees_rviz.png`（由 WSLg 渲染，Windows 侧截取；数据为**仿真数据**）

### Failure Analysis（这次真实遇到的问题）
1. **单位错误检测有盲区。** 截图中，−6.06° 被当作 −6.06 rad 时只触发了 NEAR_LIMIT 警告，整帧被拒是因为 joint3 的值足够大。幅值启发式只能抓到大于 2π 的值。→ 必须靠现场对照示教器读数和 FK 交叉校验（LIVE_ROBOT_CHECKLIST §3–4）。
2. **跨话题配对误差。** 第一版离线辨识按到达顺序把 `/joint_states_robot` 和 TCP 话题配对，mock 数据上残差为 0.264 mm（应为 0）。原因是 recorder 对不同话题的接收顺序没有保证，部分样本错了一个包。→ 改为在**同一条消息**里携带同一个包的关节角和 TCP，残差变为 0.0 mm。真机数据如果没有这个修复，会有约 0.3 mm 的伪误差混进模型判定。
3. **瞬态故障被诊断采样掩盖。** 8 ms 的跳变会被下一帧正常数据覆盖，2 Hz 的诊断看不到。→ 改为在诊断窗口内锁存最坏级别。
4. **检查器自身的误报。** 启动阶段 DDS 发现尚未完成，`count_publishers` 返回 0，导致 single_pub 误判 FAIL。→ 前 2 s 不采样。
5. **URDF 用 `-p` 传参时 rcl 解析失败**（XML 注释中的 `:` 等字符）。→ 改用 YAML 参数文件。
6. **停止时节点以 exit 1 退出。** Humble 下 SIGINT 会抛 `ExternalShutdownException`；非交互 bash 的后台作业默认忽略 SIGINT，导致 mock 没写报告。→ 两处都显式处理。
7. **mock 控制器实际约 110 Hz，不是 125 Hz**（发送后再 sleep 8 ms）。未修复，因为不影响验证；真机的周期以实测为准。
8. **WSLg 下 RViz 的文字渲染**：空格被画得极宽。→ HUD 文本不使用空格。
9. **工程环境问题**：WSL 无法使用 Windows 代理 → 离线 apt；Windows 侧 Python 改文件会产生 CRLF，破坏 bash，而 Git Bash 的 grep 检测不出来。→ 在 WSL 内检测并修复，加 `.gitattributes`。
10. HUD 的 rate 是按最近 200 个样本的窗口计算的，断流期间仍显示约 125 Hz；断流要看 `age` 和 `feed` 字段（已经能正确显示 STALE/LOST）。待改进：断流期间 rate 显示为 0。

### 当前阻碍
- 真机型号和固件版本、Link6 与法兰的关系、欧拉和四元数约定：**都需要现场数据**
- `half_close_write`（FIN）对真实控制器的影响未知

### 需要你在机械臂旁边完成
`docs/LIVE_ROBOT_CHECKLIST.md` §0–§4。最小可行集：读铭牌和固件版本 → 30005 连通性测试 → 角度与示教器对照 → 一次 120 s 的录制（多关节大范围运动）。之后的分析都可以离线完成。

### 下一步
1. 根据现场录制数据确定 CR10A 模型（运行 `crosscheck_bag.py`），更新 HARDWARE_COMPATIBILITY
2. 真机与 RViz 并排录屏，以及端到端显示延迟测量
3. M1 Case Study 草稿 → 进入 M2（MoveIt 2 规划实验室，纯仿真）

### Git
`HEAD of main` — M1: offline robot-state pipeline + read-only CR10A live path (no hardware)

---

## 2026-10-26（晚）— M1 现场只读会话 + M2 起步

### 现场（真实 CR10A，只读，全程未连接 29999、未发送任何字节）
- 网络：USB 网卡接入现场 LAN；**控制柜 192.168.201.1**（DHCP 服务器和网关 192.168.201.2）；30004/30005 有数据推送，30006 没有
- 控制器 `robot_mode` 4（DISABLED）→ 6（BACKDRIVE，操作员拖拽）；控制器时钟比主机慢 267.87 s
- **RViz 实时同步成功**（HUD 显示 LIVE_OBSERVE，状态 VALID，修复后 123 Hz）
- **模型判定**：官方 cr10 URDF 与控制器 TCP 吻合，误差 RMS 1.6–3.0 mm，max 5.54 mm（41357 个样本，覆盖六个关节的大范围运动）；CR10AF 约 43 mm。欧拉约定为外旋 XYZ，四元数顺序为 wxyz，Link6 = 法兰，base_link = user 0。详见 HARDWARE_COMPATIBILITY §7
- 真机数据和截图保存在本机 `data/live/`，**未公开**

### 现场发现并修复的问题（Failure Analysis）
1. **TCP 半关闭（FIN）导致控制器断开连接**：这是事先标为 UNVERIFIED 的风险，现场确认了 → 默认改为不半关闭（另外三层只读保护保留）
2. **约 14% 丢包**：每次 recv 只发布最新包；NAT 批量到达时会丢样本 → 改为逐包发布（大于 12 ms 的间隔从 2140 个降到 4 个）
3. **`pkill -f` 匹配到了执行它的 shell 自身**，导致构建没有执行，旧代码以 0 作除数崩溃 → `live_session.sh stop` 改用锚定的匹配模式，并等待进程退出
4. 第二段 bag 录制时机械臂没有动（人机协同的时机没对上）；**连续交叉校验 CSV 兜住了这段数据**。教训：在线节点逐包落盘很有必要
5. DHCP 下发了默认网关，可能把部分外网流量路由到现场网络（见 HARDWARE_COMPATIBILITY §7.1 的处理建议）

### M2（进行中，纯仿真）
- 离线装好 MoveIt 2.5.10 和 ros2_control；新增 `ex_moveit_config`（mock_components 硬件；关节速度按 CR10A 规格设置）和 `ex_planning_lab`（3 个场景、benchmark、execute_demo、胶囊 clearance 模型）
- 冒烟测试：move_group 正常，两个控制器 active；RRTConnect / RRTstar / PRM 规划成功；**BITstar 在 MoveIt 2.5 中没有注册**（"Unknown planner"），已移除
- 单元测试 **80/80 通过**
- 已知问题（下一步）：试跑时所有 clearance 都是 −0.079 m，怀疑是 base_link 胶囊与桌面重叠，待修复后再跑完整基准

---

## 2026-10-27 — M2 规划实验室：基准实验与方法学修复

- **run01 作废**：存在多个残留的 move_group 实例，加上碰撞检查步长过粗（8.2°），导致大量假失败（92 次 path invalid、47 次 goal invalid）。详见 M2_PLANNING_LAB §6.3
- 修复：`scripts/m2_session.sh`（保证只有一个 move_group）、benchmark 启动时断言、`longest_valid_segment_fraction` 0.005 → 0.00035、查询 clearance ≥ 3 cm、clearance 改为区间指标
- **run02（有效）**：400 次规划，错误码只有成功和超时。RRTConnect 与 SBL 均为 39/40；单向 RRT / RRT\* 在点目标下几乎全部超时；狭窄通道（window）中 PRM 为 0/15；BiTRRT 在默认参数下为 0/40（原因待查）
- 跨里程碑结论：规划路径的余量（1–2 cm）≤ 真机模型误差（最大 5.5 mm）的数倍 → 真机执行前必须设置 padding
- 单元测试 84/84；图表在 `assets/m2/`（仿真数据，可以公开）
- 另外：笔记本夜间休眠导致一次试跑的墙钟时间出现 13 小时跳变；每次规划的耗时由 MoveIt 单独报告，不受影响
