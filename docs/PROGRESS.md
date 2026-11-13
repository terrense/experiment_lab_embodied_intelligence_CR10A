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

---

## 2026-10-27 — 发布真机数据 + CR10A 运动学对齐

- 经所有者同意，公开了整理后的真机证据：截图（`assets/m1_live/`）、交叉校验日志、模型判定结果（`docs/evidence/m1_live/`）
- **运动学对齐**（`docs/CALIBRATION.md`）：45 参数模型，正则化后可辨识秩 25；跨会话留出误差从 3.1–3.4 mm 降到 **0.002–0.011 mm**；经独立的 bag 管线验证，位置误差 0.0002 mm，再加上 0.207° 的法兰旋转修正后姿态误差 0.00004°。消融实验：只修零点能解释约 30% 的偏差，其余来自连杆几何
- 解读：复现的是**控制器的标定模型**，不是外部测量意义上的绝对精度
- 所有 launch 默认改为 `calibrated:=true`；单元测试 90/90 通过；集成测试 run07 全部阶段通过
- 踩坑：XML 注释里不能出现 `--`（生成文件头写了 `--write-urdf`，导致 xacro 解析失败）；生成器自己依赖被生成的文件，出现先有鸡还是先有蛋的问题；`crosscheck_bag` 默认算到 Link6，漏掉了 1.5 mm 的法兰偏移，表现为恒定的 1.5 mm 误差

---

## 2026-10-27（下午）— M4 MuJoCo 被控对象 + 跟踪实验

- MuJoCo 3.3.7 被控对象接到 ros2_control 下面，MoveIt 及以上一行不改；单元测试 100/100
- 失败分析（详见 M4_MUJOCO）：线程内的 import 遮蔽（物理线程死掉但节点看起来正常）；JTC 没设容差时会把"没动"判为成功；按到达时间做速度差分引起发散；定位失败的运行被静默丢弃；**33 个孤儿监视进程把负载推到 26，run02 作废**
- run03（干净环境）：误差约与 1/伺服带宽 成正比（8→16 Hz：9.2→4.6 mm）；4 Hz 全部中止；已知负载被完全补偿（模型假设导致的）；60 次执行中有 7 次间歇性 −4 中止，原因未查明（mock 上也出现过）
- M2 复测（run03）比 run02 慢 2.7 倍，原因未能证实（复测时电池 7%）→ 跨时段计时不可比，规划器排名结论不变

## 2026-10-28: first program-controlled motion of the real CR10A (ex_dobot_motion)
- Done on site, with the owner at the e-stop and unit authorization. Two dry-runs first, which caught and corrected my wrong claim that the TCP never goes below the start; then two executions:
  - run01: 22 s, unusable because of the timestamp-precision bug.
  - run02: 84.7 s, 241 moves, clean. Returned to start, disabled, no error.
- Evidence: `docs/evidence/live/excitation_run02/`. Fix: full-precision CSV + regression test (ex_dobot_motion 32/32).
- Controller quirk recorded: a garbage packet at the ENABLE transition.
- Next: dynamics-ID analysis (validate on MuJoCo synthetic data first, then apply to run02).

## 2026-10-28: dynamics identification (M4.5), see docs/DYNAMICS_ID.md
- `ex_robot_state/core/dynamics.py`: inverse-dynamics regressor that is linear in the parameters, verified against MuJoCo `mj_inverse` (5 tests).
- `evaluation/dynamics_synthetic.py` (known truth) + `evaluation/dynamics_id.py` (M0/M1/M1s/M2, time split, hyper-parameters chosen on validation).
- Synthetic: armature, Coulomb friction and the mass scale are recovered; viscous friction is NOT recovered at these speeds.
- Real run02: URDF masses about 1.64× too light (matches the 40 kg datasheet); Coulomb 10–14 N m on J1–J3; held-out torque VAF 96–99 %.
- Blocker for full identification: a richer excitation needs a wider motion envelope, which needs the owner's OK and an on-site session.

## 2026-10-28: M5 learning, wall task (docs/LEARNING.md)
- Reach-around-the-wall env on the M4 plant; collision is measured with the M2 capsules (mesh distance queries were unreliable, see failure analysis).
- Baselines first (50 eval episodes): DLS 4 % success / 96 % collisions; APF 44 % / 0 %.
- PPO (SB3, isolated venv, CPU, 3 M steps): 3 seeds reach 58 / 64 / 80 % success with 0 collisions. Every seed beats APF.
- Reward v1 taught PPO to crash; fixed with a potential-based reward and a regression test.
- Hands-on launchers + Chinese guide (`hands_on/`); README is now a visual tour of all stages.

## 2026-10-29: M5 window scene + generalisation (docs/LEARNING.md)
- Window (narrow passage, mesh point-cloud collision): DLS 6 % / 76 % collisions, APF 0 %, PPO (5 M) 0 %, RRT-Connect + path following 38 % / 2 % (40 % planned within 60k checks).
- Generalisation, 50 paired episodes, 3 PPO seeds each: one-wall PPO 67 % → 21 % on moved walls; random-wall PPO (wall observed) 77 % in distribution (APF 64 %), 21 % success / 36 % collisions on taller+wider walls (APF 50 % / 2 %).
- Bug found before training: `step()` read the wall size as the clearance when the wall was in the observation; fixed + tests.
- Commands: `python -m ex_learning.train_ppo --task wall_rand --seed {0,1,2} --steps 5000000`, `python -m ex_learning.eval_ppo --model ... --task {wall_rand,wall_ood,wall_obs}`, `python -m ex_learning.planner --task window --exec-steps 1500`, `python3 evaluation/plot_generalisation.py data/reports/learning --out assets/learning/generalisation.png`.

## 2026-10-29: M5 safety filter (CBF) on top of PPO (docs/LEARNING.md)
- Question: can a safety filter stop the 36 % OOD collisions without killing success? Two filters on the generalist wall_rand policy (3 seeds, 50 paired episodes):
  - naive additive repulsion (d0 0.10, k 0.02): OOD 15 % success / 17 % collisions, but original wall 71 -> 43 % success (fights the policy).
  - CBF projection (margin 0.05, alpha 5): OOD 21 % -> 31 % success, 36 % -> 5 % collisions; in-distribution 77 % -> 81 % and 4 % -> 0 % collisions; original unchanged 71 %.
- New modules: ex_learning/safety_filter.py (safe_policy, cbf_policy), eval_ppo_safe.py, run_safe_eval.py, sweep_safe.py; 4 tests in tests/test_safety_filter.py.
- Commands: python -m ex_learning.run_safe_eval; python3 evaluation/plot_safety_filter.py data/reports/learning --out assets/learning/safety_filter.png.

## 2026-11-02: M5 research-state brief for external analysis (docs/RESEARCH_STATE.md)
- Wrote a self-contained, stand-alone snapshot of the whole project (M1 → M5) meant to be fed to
  an external paper-literate reader (ChatGPT / a colleague) so it can propose paper-grounded next
  steps without reading the repo: hardware + access constraints, per-stage measured results, the
  learning setup in detail, the honest failure log, and 12 numbered open questions grouped into
  (A) learning through clutter, (B) safe RL / safety filters, (C) sim-to-real, (D) planning gaps,
  (E) perception, (F) prior art to adopt.
- Push note: WSL NAT cannot reach the Windows Clash proxy (127.0.0.1:7890), so this commit was
  pushed from Windows git (C:\Program Files\Git) instead of from WSL.

## 2026-11-04: external source audit -> defects verified, fixed, and the research plan written
- An external review of the archived code (CR10A_Research_Design_CN.md, 2026-11-02) alleged 6 P0
  and 4 P1 defects. Every one was independently re-derived before anything was changed; several
  severities were corrected (the dead limit mask is P2, not P0: it fires 0 times in >1M substeps
  across all archived policies; the sampled-clearance defect does NOT flip the collision sign on
  the real CR10A capsules, it over-states the clearance by up to 12 mm against a 20-50 mm margin).
- Fixed (commit 5bf50aa, 175 tests pass, 15 new regression tests):
  P0-A bounded projection onto box INTERSECT half-space (learning/ex_learning/qp_filter.py) with
  the legacy filter kept as cbf_policy_legacy for the archived JSON; P0-B the joint-limit mask is
  captured before q_ref is overwritten; P0-C conservative_geometry makes the capsule clearance a
  rigorous lower bound; P0-D opt-in per-substep collision checking; P1-b the dynamics preset is a
  config field and is recorded on the env. All defaults keep the LEGACY behaviour.
- New: docs/RESEARCH_PLAN.md - the main line (execution-aware safety filtering), the verified
  audit table, the frozen evaluation protocol (seeds 1000-1049 are now historical dev data; a new
  untouched test range is 20000-20199), the B0-B8 method ladder, the six-week plan with decision
  gates, and the falsification criteria stated BEFORE the results.
- Honesty edits: "success roughly doubles" -> "+10 points, +48 % relative, still below APF's
  50 %"; the window comparison is labelled not budget/privilege-matched; the CBF is a kinematic
  filter with audited defects, not a safety guarantee.
- Next: the corrected evaluator (evaluation/independent_collision.py) and the episode manifest.

## 2026-11-04 (later): corrected evaluator, independent checker, result manifest (P1-a, P1-d)
- New evaluation/independent_collision.py: a second checker that does NOT reuse the environment's
  distance code - mesh SURFACE samples instead of the enclosing capsules, an exact point-to-AABB
  distance, a penetration depth for containment, and a self-collision check over the link pairs
  the MoveIt SRDF does not disable. Validated against the environment: capsule axes coincide to
  1e-6, every mesh point lies inside its capsule (max excess 0.000000 m).
- New evaluation/episode_manifest.py: the frozen protocol. Seeds 1000-1049 are now historical DEV
  data (they produced the published numbers and were inspected while writing them); the
  confirmatory range is a new, untouched 20000-20199.
- New evaluation/compare_evaluators.py: replays the SAME recorded actions through three
  detectors, so the physics is identical and only the detector differs. Its self-consistency
  assertion caught two of its own bugs, most importantly that the episode SAMPLING depended on the
  evaluator (the corrected clearance is lower, so the rejection sampling accepted different
  start/goal configurations - the two evaluators were not comparing the same episodes at all).
- P1-d fixed: every result JSON now carries a full manifest (filter settings, env config, plant
  preset, collision-evaluator variant, seed range, git commit, SHA-256 of the checkpoint, episode
  manifest hash).
- P1-a prepared (opt-in, the legacy reward stays the default): discount-consistent shaping
  gamma*Phi(s')-Phi(s) with the identity sum gamma^t F_t = d_0 - gamma^T d_T as a regression
  test, and an optional joint-limit penalty the legacy wall reward dropped.
- Measured so far: the environment's collision detector is sound but LOOSE. It reports "the
  enclosing capsule touched the wall" while the mesh may still be 13 cm away (home pose: capsule
  clearance 0.054 m vs mesh clearance 0.130 m). The published collision numbers therefore measure
  a different physical event than the paper's wording suggests.
- 189 tests pass.

## 2026-11-04 (evening): P1-c measured - the filter leakage is real but negligible
- evaluation/dynamics_id.py gains --split-before-filter: split into train/val/test FIRST and
  filter each part separately, with a 0.2 s guard interval at the internal boundaries, so the
  held-out test part no longer sees the training part through the zero-phase filtfilt.
- Measured on the archived run02: the result moves by <= 0.1 VAF percentage points
  (M1 97.2/97.3/98.5 -> 97.2/97.3/98.4, M2 97.5/97.4/98.7 -> 97.5/97.4/98.6), mass scale
  k = 1.636 and the Coulomb values are unchanged. The leakage is real as a mechanism and
  irrelevant in magnitude. The corrected report is written to a NEW file; the archived one is
  untouched.
- Two bugs of my own were found and fixed on the way, both by reproducibility checks:
  1. the concatenation of separately filtered segments has a TIME GAP, so np.gradient over the
     concatenated array invented huge acceleration spikes at the joins and destroyed the fit
     (the derivative must be computed inside each segment);
  2. I transcribed qdd = d(filtered velocity)/dt as d(filtered position)/dt, which silently
     changed the LEGACY path and did not reproduce the archived JSON. The legacy path now
     reproduces data/reports/dynamics/run02_id.json exactly, and a regression test pins it.
- New: ros2_ws/src/ex_robot_state/test/test_dynamics_id.py (4 tests). 193 tests pass.

## 2026-11-04 (night): P0-C re-measured with an exact reference; the same defect in the M2 metric
- An independent verification measured the P0-C sampling defect against an EXACT segment-AABB
  reference (ternary search, validated against a 200001-point sweep). It corrected my own first
  conclusion: the sign DOES flip, it is just rare and small.
  * per-pair over-statement on the real CR10A capsules and the real wall box: mean 0.006 mm,
    p99 0.14 mm, max 6.95 mm (adversarial 7.08 mm on Link2, where the true value is already
    deeply negative);
  * missed collisions (repo reports safe, the geometry collides): 10/3000 (0.33 %) of
    near-boundary configurations at margin 0, 1/3000 (0.03 %) at the filter's 2 cm margin,
    0/3000 at 5 cm; the error at a flip is 0.06-0.42 mm;
  * the Lipschitz correction misses 0/3000 at every margin.
  My earlier claim that the sign never flips on the real capsules was too strong; the test
  comment and the plan table now say what was measured.
- The SAME defect exists in the M2 planning metric (ex_planning_lab/core/capsules.py,
  samples=16): the sampling error is L/30 = up to 25 mm for Link2, i.e. LARGER than the 1-2 cm
  path margin the M2 report quotes. capsule_box_distance gains a conservative=True option that
  subtracts it; the default stays legacy so the archived M2 numbers reproduce, and
  docs/M2_PLANNING_LAB.md now flags that the real-robot padding conclusion must be recomputed
  with the corrected value (the direction is: MORE margin, not less).
- New: the bounded ALL-PAIR kinematic QP (learning/ex_learning/qp_filter.py::solve_bounded_qp,
  scipy SLSQP, bounds inside the optimisation) and the B3 filter
  (safety_filter.cbf_policy_qp) with 5 regression tests; it never returns a violating command
  and fits inside the 50 Hz control rate.
- 199 tests pass.

## 2026-11-05: Week-1 gate PASSED - the published M5 table is not an artefact of the evaluator
- Every defect of the 2026-11-02 external review was independently re-derived and MEASURED; the
  full report is docs/EVALUATOR_AUDIT.md.
- The gate: evaluation/compare_evaluators.py replays the RECORDED ACTIONS of every episode
  through the corrected environment, so the physics is identical and only the detector differs
  (the driver asserts this - replaying in the legacy env must reproduce the legacy verdict).
  Result over 3 seeds x 3 tasks x {raw, CBF} x 50 dev episodes: the corrected evaluator changes
  the verdict in at most 5/50 episodes, and only in the raw-policy OOD cells; 16 of 18 cells are
  identical. The published table stands.
- The audit harness was validated by reproducing the archived gen_safe JSON EXACTLY (success
  0.30, collision 0.04, final dist 0.0289 m): the audited code IS the code that produced the
  published numbers.
- Two of my own conclusions were corrected by the independent measurements: P0-C DOES flip the
  collision sign, but only in 0.33 % of near-boundary configurations (0.03 % at the filter's
  2 cm margin, 0 % at 5 cm); and P1-c's leakage moves the held-out VAF by <= 0.1 points, not by
  3-5 (my earlier number was an artefact of a transcription error of mine, now caught by a
  reproducibility test).
- A finding the review did not anticipate: the collision detector is sound but LOOSE - it answers
  "did the enclosing capsule touch the wall", and the capsule can be in contact while the mesh is
  still 13 cm away. The same looseness is in the M2 planning metric (up to 25 mm of sampling
  error, larger than the 1-2 cm path margin the M2 report quotes), so the real-robot padding
  conclusion must be recomputed in the direction of MORE margin.
- 199 unit tests pass.

## 2026-11-05 (later): the collision METRIC turns out to be the biggest finding of the audit
- The independent mesh-based checker, validated against MuJoCo's own compiled mesh vertices
  (0.000 mm agreement - the earlier "inside its capsule" check was NOT sufficient, because it
  transforms the capsule axis and the mesh points with the same frame and a wrong frame cancels
  out), reports on the real OOD-wall episodes:
    * the legacy evaluator calls a collision in 19/50 episodes (38 %);
    * the mesh checker calls a collision in 0/50;
    * in those 19 episodes the MESH was still 24-65 mm away (median 45 mm).
  So the published "36 % out-of-distribution collisions" are, physically, false positives of the
  conservative capsule model: the arm never touched the wall, the safety ENVELOPE did. The
  success rates are unaffected; the wording has to change, and the mesh clearance is now the
  number that goes into the paper.
- The window comparison, MATCHED (same env, collision model, pools, success criterion, 30 s
  deadline for everybody, and the Cartesian goal for everybody - the planner realises the goal
  itself instead of using the privileged q_goal): DLS 8 %/70 %, APF 0 %/6 %, RRT-Connect 38 %/6 %.
  The conclusion survives the matching.
- B4 baseline sweep (the bounded all-pair QP with a fixed margin, tuning seeds 0-19, 3 policy
  seeds). On the OOD walls the QP has 0 % collisions at every margin >= 2 cm, while the
  single-constraint projection still collides (3.3 % at 2 cm) and gets MORE dangerous as it
  becomes more conservative (3.3 % at 10 cm): a single constraint can only push away from the
  closest obstacle and may push the arm into another one. In distribution the QP is insensitive
  to the margin up to 6 cm (72-73 %) and degrades at 15 cm (43 %).

## 2026-11-05 (evening): the manuscript enters the repo, and the command channel is MEASURED
- The paper is now in the repository as `paper/manuscript.md` (Markdown, because neither pandoc nor
  a LaTeX toolchain is installed here - `paper/README.md` explains how to build the PDF). It is the
  authoritative copy: every number in it has a committed JSON behind it. The manuscript states the
  collision-metric redefinition (the published OOD collision rates count the safety ENVELOPE, not
  the arm), the matched window protocol, the Week-1 audit gate, and the B0-B8 ladder.
- The research main line was then implemented as B5, the EXECUTION-AWARE filter
  (learning/ex_learning/execution_model.py + safety_filter.cbf_policy_exec):
  * the plant executes a command through a CHANNEL - the command is integrated into a position
    reference, the reference is clipped at the joint limits, and a bandwidth-limited servo
    (omega_n = 2*pi*8 rad/s, zeta = 0.9) tracks it. A kinematic filter that treats the requested
    velocity as the executed velocity therefore omits exactly the braking dynamics;
  * evaluation/measure_execution.py measures that channel on the SIMULATED plant: the arm coasts
    **14.9 mrad (0.015 rad) at a 1 rad/s command** after the command is cut, and the identified
    second-order model predicts the position over 5 policy steps with an RMS error of **4.6 mrad**
    (omega 37.7 rad/s, zeta 0.9, v_gain 0.8). The prediction error is flat within +-20 % of the
    identified omega, so the filter does not depend on an exact plant identification;
  * the filter constrains the PREDICTED MOTION instead of the command: the prediction is expanded
    to first order in the command, dQ_k(v) ~= dQ_k(0) + G_k.v, where dQ_k(0) is the displacement
    with a ZERO command - the coast. Every (link, obstacle) pair and every horizon sample becomes
    one linear constraint in the command, so it is still ONE bounded program with the command
    bounds inside the optimisation;
  * qp_filter.solve_bounded_qp_relaxed handles the case where the program is genuinely
    infeasible (the arm is already coasting toward an obstacle faster than any command can stop
    it): it returns the closest command and REPORTS the violation. A relaxed constraint is a
    violation, never a certificate;
  * the ablation `account_execution=False` drops the coast term and keeps the lookahead, so the
    paper can say whether the result comes from the lookahead structure or from the execution
    model.
- 207 unit tests pass (65 learning + 77 ex_robot_state + 32 ex_dobot_motion + 17 ex_planning_lab +
  16 ex_mujoco_sim; 7 new in learning: the coast, the affine expansion, the relaxed solver, the bounded
  command of the filter, and the coast-aware filter being at least as conservative as the
  execution-blind one).
- RUNNING (results appended below when they land): the frozen-range final benchmark
  (3 seeds x 3 tasks x {raw, cbf_legacy, qp} x 50 test episodes, three detectors) and the B5
  margin sweep on the TUNING seeds 0-19.

## 2026-11-05 (night): the execution gap is measured, and falsification criterion 1 is TRIGGERED
- The measurement behind the main line (`evaluation/execution_gap.py`): at every control step of a
  real episode, was the filter's OWN reasoning satisfied, and did the plant then end the step
  below the enforced clearance margin? Out-of-distribution walls, 20 episodes, 6000 control steps:
    margin   bounded kinematic QP   exec-aware (no coast)   exec-aware (coast)
      0.5 cm        3.6 %                 0.1 %                0.6 %
      1.0 cm        8.0 %                 0.1 %                0.4 %
      2.0 cm        9.2 %                 0.2 %                0.6 %
      4.0 cm       32.4 %                 1.3 %                4.8 %
  The kinematic filter's constraints hold while the arm still ends the step below the margin; the
  command-level prediction is optimistic by 12 mm in the median and 146 mm in the worst step, and
  the MORE conservative the filter is set, the MORE often its own claim fails.
- The attribution is against our expectation and is recorded as such: the gain comes from the
  LOOK-AHEAD structure (constraining the predicted motion over a horizon instead of the command's
  rate), NOT from the explicit execution model. Dropping the coast term changes the frontier not
  at all and makes the filter's own claim consistently MORE valid.
- **Falsification criterion 1 of docs/RESEARCH_PLAN.md is TRIGGERED**: the bounded all-pair QP
  matches the execution-aware filter on collision rate (both 0 % from a 5 mm margin), does not
  lose on completion (28 % against 30 %, one episode in sixty) and is ~8x cheaper per step. The
  claim that survives is narrower and is what the manuscript now says: a command-level guarantee
  is not a motion-level guarantee, and the fix is the constraint structure, not the plant model.
- The QP frontier sweep was RE-RUN with one consistent margin list for all three policy seeds,
  because an earlier summary mixed freshly written and stale sweep files. The corrected numbers:
  the QP at margin 0 collides in 3.3 % of the episodes (it is NOT safe with no fixed clearance),
  and is collision-free from 5 mm upwards at 28 % completion; the single-constraint projection is
  non-monotonic (0 % at a 6 cm margin, 3.3 % again at 10 cm). The corrected table is in
  docs/RESULTS.md section 7; the stale "35 % / 0 %" claim is withdrawn there and in the
  manuscript.
- 207 unit tests pass (65 learning + 77 ex_robot_state + 32 ex_dobot_motion + 17 ex_planning_lab
  + 16 ex_mujoco_sim).
- The manuscript (paper/manuscript.md) is updated: abstract, 5.0 (what was implemented and what
  it is not), 7.4-7.7, discussion and limitations all reflect the measured results, including the
  parts that weaken the proposed method.

## 2026-11-05 (late): the CONFIRMATORY run on the frozen test range
- The test range 20000-20199 (never inspected before) is now measured with THREE detectors per
  episode: the legacy envelope detector, the corrected detector, and the independent mesh
  checker. Script: evaluation/final_benchmark.py; report:
  data/reports/learning/final_benchmark.json; episode manifest hash a995f097b28148c2. 3 policy
  seeds x 50 episodes per cell.
    method                       task          succ   legacy  corrected   mesh   mesh clearance
    unfiltered                   fixed wall   78.0%    0.0%      0.0%   0.0%        118.7 mm
    unfiltered                   random walls 83.3%    0.0%      0.0%   0.0%        117.6 mm
    unfiltered                   OOD walls     36.0%   38.7%     40.7%   0.0%         55.9 mm
    archived single-constraint   OOD walls     46.0%   10.7%     11.3%   0.0%         89.2 mm
    bounded all-pair QP (4 cm)   OOD walls     45.3%    0.0%      0.0%   0.0%         88.0 mm
    bounded all-pair QP (4 cm)   all           68.4%    0.0%      0.0%   0.0%        111.2 mm
- The archived OOD envelope-collision rate REPRODUCES on untouched data (38.7 % against the
  published 36 %), and the mesh checker finds NO real contact in any cell - the collision-metric
  finding holds on data that was never used to reach it.
- The bounded all-pair QP DOMINATES the archived filter: the same completion (45.3 % against
  46.0 % out of distribution) with 0 % against 10.7 % envelope collisions. Removing the
  single-constraint defect costs nothing in completion.
- The filters also roughly double the physical distance the arm keeps from the wall out of
  distribution (median mesh clearance 55.9 mm unfiltered -> 88-89 mm filtered).
- The independent checker was optimised (one pass over the link pairs instead of two, each link's
  world points transformed once per configuration). The optimisation was verified to be BIT-IDENTICAL
  against the previous implementation on real trajectories (max numeric difference 0.000e+00, all
  booleans and pairs equal); the speed-up is 1.2x.

## 2026-11-06: the confirmatory run CHANGES the conclusion - and the write-up with it
- The five-method confirmatory run on the frozen test range (20000-20199, 3 seeds x 50 episodes
  per cell, three detectors) landed: data/reports/learning/final_benchmark_full.json.
    method                       OOD succ   OOD legacy  OOD corrected   mesh   ALL succ
    unfiltered                    36.0%       38.7%         40.7%       0.0%      65.8%
    archived single-constraint    46.0%       10.7%         11.3%       0.0%      68.7%
    bounded all-pair QP (4 cm)    45.3%        0.0%          0.0%       0.0%      68.4%
    bounded all-pair QP (0 cm)    54.0%        8.0%         19.3%       0.0%      71.8%
    execution-aware (1 cm)        54.0%        0.0%          0.0%       0.0%      71.8%
- **Per policy seed (OOD, 50 episodes each):**
      seed 0:  QP(4cm) 50 % / 0 %    QP(0cm) 72 % / 22 %    exec 72 % / 0 %
      seed 1:  QP(4cm) 44 % / 0 %    QP(0cm) 46 % / 20 %    exec 46 % / 0 %
      seed 2:  QP(4cm) 42 % / 0 %    QP(0cm) 44 % / 16 %    exec 44 % / 0 %
  So in EVERY seed the execution-aware filter reaches exactly the completion of the
  unconstrained-margin QP while that QP collides in 16-22 % of the episodes - that is the
  established result. Its completion advantage over the SAFE fixed-margin QP (+22/+2/+2 points
  against an 8-point between-seed spread) is NOT established and is not claimed.
- **This reverses the conclusion I recorded earlier today** ("falsification criterion 1 is
  TRIGGERED"). It was based on the 20-episode tuning sweep, which was underpowered: the sweeps
  were re-run with 50 episodes per cell and the single-constraint projection was added, and the
  criterion is now recorded as PARTIALLY triggered (collision rate matched, completion not
  separated, latency against the method, and the execution-model mechanism unsupported by the
  ablation). The plan, the manuscript's abstract/discussion/limitations and RESULTS.md are all
  corrected to match.
- **A protocol flaw found and documented:** the TUNING seeds 0-19 lie inside the TRAINING range
  0-999, so the operating points were chosen on scenarios the policies have already seen. On
  those seeds the two filters are indistinguishable (32.0 % against 31.3 %); the confirmatory
  test range is where they separate. The manifest and the plan now say so, and a future protocol
  must place the tuning split outside the training range.
- The 50-episode sweeps confirm the single-constraint projection's NON-MONOTONICITY (5.3 % ->
  1.3 % -> 4.7 % collisions at 0, 6 and 10 cm) and show the bounded QP is collision-free from a
  5 mm margin upwards.
- Robustness on the IDENTIFIED plant (--dynamics identified_run02: 40.6 kg of link mass against
  the placeholder's 24.8 kg, matching the datasheet): the same pattern, exec 54.7 % against the
  QP's 45.3 % out of distribution, both collision-free, mesh 0 % everywhere.
- Figure 5 (assets/learning/frontier.png) is redrawn from the 50-episode sweeps; the ablation is
  no longer drawn (it lies exactly on the B5 curve) and the note in the panel says so.

## 2026-11-06 (evening): the statistics the protocol promised, and the seed count it needs
- **Compute assessment (the standing rule, stated before the experiments).** Every experiment is
  assessed first; if the laptop is enough, it runs on the laptop. The assessment table is in
  docs/REMOTE_COMPUTE.md. The honest finding: the H20's VRAM is NOT the binding constraint
  anywhere in this plan - the MuJoCo plant is CPU-bound and the policies are small MLPs, so the
  server's value is its CORE COUNT (many seeds in parallel), which is exactly what the
  statistical weakness below needs. The one place a GPU genuinely matters is the residual-RL arm
  (B7) in Week 5.
- **Remote scaffolding prepared** (scripts/remote/{setup_env,sync,train_seeds,fetch_results}.sh +
  remote_config.example.sh). The SSH target and the remote paths live in a git-ignored local
  copy; nothing about the owner's infrastructure is committed. The motion-capable package
  (ros2_ws/src/ex_dobot_motion) is deliberately EXCLUDED from the sync: a compute node must not
  hold a copy of the code that can move a robot.
- **The xacro expansion is now cached** (data/cache/ex_cr10_calibrated_1.urdf, 7.8 kB, committed,
  byte-identical to what xacro produces). It used to run on EVERY environment construction, i.e.
  every episode of a sweep; it is 18x faster now, and a remote node needs neither ROS nor xacro.
- **evaluation/stats.py** - the statistics section 6.3 of the manuscript promised and that did not
  exist: the paired scenario bootstrap, the two-level (seeds, then scenarios) hierarchical
  bootstrap, the naive interval kept only so the report can show how much it over-claims, the
  between-seed spread, an explicit separability verdict, and the Clopper-Pearson bound with its
  independence warning. Every function returns its assumptions. 13 new tests (220 in total); the
  key one constructs data with a scenario effect shared across the seeds and a per-seed shift of
  the difference, and shows the hierarchical interval is more than twice as wide as the naive one.
- **The seed count is the bottleneck.** Three training seeds cannot separate an 8-point mean
  difference from an 8-point between-seed spread, and the confirmatory result hinges on exactly
  that. So six more seeds are training locally (5M timesteps, the archived configuration) and
  twelve more can be trained on the server when the owner connects it. Each completed seed is
  immediately usable; the confirmatory statistics re-run with whatever is available.
- **Manifest v2**: the tuning split moves OUT of the training range (v1 used seeds 0-19, inside
  train 0-999 - the protocol flaw found while checking the confirmatory result), and a second
  untouched range (30000-30199) is added for replication. v1 is frozen verbatim, so its hash
  (a995f097b28148c2, quoted in the committed reports) still reproduces - verified.

## 2026-11-08/09: the two things the filter does not do, measured
- **Compute assessment first (the standing rule).** The two experiments below are CPU-bound
  (MuJoCo at 1 kHz, a bounded program per control step); neither touches the GPU, so both run on
  the laptop. The one thing that changed is the PROCESSING: the B6 cells are independent, so they
  run in six parallel processes instead of one (the machine has 16 cores and the training job
  takes 12). The H20's core count is still the resource that would matter for more seeds.
- **The stopping tail is now measured, not asserted** (`evaluation/stopping_rollout.py`). The
  states are sampled from the EXECUTION-AWARE FILTER'S OWN OUTPUT - where the clearance is at
  least the filter's 10 mm margin by construction - because the unfiltered policy's states are
  already in collision half the time and the question is meaningless there. The command is cut
  to zero and the plant is rolled on with every physics substep recorded. The result: the arm
  keeps moving and can lose **87 mm of clearance** (p95 38 mm) after the cut, **8.7x the
  filter's own margin**, and in **35 %** of the states the tail brings it CLOSER to the
  obstacle. The motion is over after 200-500 ms; the filter's look-ahead is 100 ms. The coast
  itself is modelled well (measured/predicted 0.99997), so the horizon is the problem, not the
  coast. This is the empirical answer to the "stopping verification" the manuscript had listed
  as not implemented.
- **The nonlinear re-check.** The filter's constraint is a first-order expansion. Rolled out
  nonlinearly: the expansion stays within **8.5 mrad** of the model's own output, so the
  LINEARISATION is not the problem - but the MODEL is: its error at the same horizon is **22.3
  mrad** on the states the filter produces, against the **4.57 mrad** it was fitted on (a
  random-command rollout on the reach task). The identified model is a good model of the
  channel in general and a poor model of it where the filter depends on it.
- **The uncertainty budget, separated** (this is what 5.5 asked for). The geometric tube is
  MEASURED: the environment scores clearance with the enclosing capsules, the truth is the mesh,
  and the gap is **one-sided** - the capsule is smaller in **1512 of 1512** samples, by 49 mm at
  the median (126 mm worst). So the geometry costs the margin nothing; the PREDICTION error and
  the STOPPING TAIL eat it. The part that looked like the hard modelling problem is the part
  that is already safe.
- **Two bugs found and fixed while measuring.** (a) The mesh checker owns its own MuJoCo model
  instance, so its pose has to be set explicitly - without that the geometric comparison was made
  at the wrong configuration and produced nonsense (both signs of the error). (b) The
  manifest tests still encoded the version-1 protocol after the manifest moved to version 2
  (the tuning split moved out of the training range, a replication range was added): the two
  stale assertions are now written against version 2, and version 1's contaminated tuning split
  is asserted separately so the history stays on the record.
- **The planner was made tractable, without changing a single plan.** `JointRRTConnect._extend`
  searched the tree for the nearest node with a Python loop, which is O(N) per extension in
  Python: a failed 20k-check search built a tree of thousands of nodes and took **1711 s**. The
  search is now a numpy `argmin` over the same list - the same node is selected (argmin returns
  the first minimum, exactly as the comprehension did) and the random number stream is
  untouched. The archived window run is re-checked against it.
- **The vectorised planner is verified against the archived run, episode by episode.** The
  re-check of `learning/ex_learning/planner.py --main` on the window task reproduces
  `data/reports/learning/rrt_window.json` EXACTLY: the same eight episodes, the same solved
  flags, the same number of collision checks (26951, 60000, 60010, 27165, 7598, 47942, 60003,
  60006) and the same number of waypoints. A regression test pins the invariant that matters
  (the vectorised nearest-node search selects the node the original Python loop selected, and the
  planner is deterministic for a fixed seed). NOTE: `rrt_window_exec30s.json` is NOT the
  comparison file - it was produced by `matched_window.py`, which realises the goal from the
  PRIVILEGED goal pool, so it is a different protocol and a different plan set.
- **B6 is done: the planner arms (the plan's Week-5 arm, and section 5.6's question).** The
  planner is given the scene geometry, the start configuration and the CARTESIAN goal, and has
  to realise the goal into a joint configuration itself with a DLS IK - no privileged goal
  configuration, no goal pool. It plans **42 of the 50 out-of-distribution episodes (84 %)** in a
  median of **1.5 s** (worst 16 s), realising the goal to 0.033 mm. One plan per episode, shared
  by the three planner arms; the planner runs in the rigorous (lower-bound) clearance model.
  - The **harness is validated against the confirmatory benchmark**: the 6 s column of the B6
    table is the same measurement code on the same episodes, and it reproduces section 7.7 cell
    for cell - 36.0 %, 45.3 % and 54.0 % completion, and 38.7 % / 40.7 % envelope collisions for
    the unfiltered policy. The six-arm comparison rests on the same code as the published result.
  - **The filter works across proposal sources and is REDUNDANT on a good one.** On the planner
    it changes nothing (84.0 % without it, 84.0 % with the execution-aware filter, 80.0 % with
    the bounded QP - slightly worse, because the QP brakes a follower that was already safe). On
    the reactive policy the same filter turns 38.7 % into 56.7 % and 38.7 % envelope collisions
    into 0 %. The filter's value is a property of HOW BAD THE PROPOSAL IS, not of the filter.
  - **On this task the learned policy adds nothing on top of a planner** handed the same
    information (84.0 % against 56.7 %) - a statement about this task, not about learning in
    general. The planner is also SLOW (4 % at 6 s against the policy's 54 %), so it needs a
    deadline five times longer, and it fails to plan 16 % of the episodes.
  - **A statistical caveat that is in the section, not hidden:** the planner arms are identical
    in every training seed, because a plan does not depend on a policy. They have no
    between-seed variance and cannot enter a seed-level bootstrap as the policy arms do.
- **The claim checker grew to 156 claims** (from 99), every one of them re-derived from the
  committed JSON: the two new manuscript sections, the B6 table and the harness cross-check are
  all checked, so a number that drifts away from its source fails the test suite.
- **Five training seeds, and the statistical verdict changes in one direction only.** Two more
  PPO seeds were trained with the archived configuration (seed 3: 66 % on wall_rand, seed 4:
  80 %, seed 5: 80 % - about 70 min each now that the machine is not suspending) and the same
  analysis was repeated on the same episodes.
  - **The completion verdict does not change, but its REASON does.** With three seeds the
    verdict was "too few seeds to resolve it"; with five it is the substantive one: the
    hierarchical interval for execution-aware - QP (4 cm) is [+1.2, +15.2] points (mean +6.8,
    per-seed differences +22, +2, +2, +6, +2) and the between-seed spread of the DIFFERENCE is
    8.7 points - the same order as the difference. The naive interval (every cell independent)
    is [+4.0, +10.0], 2.3x narrower, and is the one a per-episode analysis would have reported
    as a settled win. The completion advantage over the safe fixed-margin filter is therefore
    NOT established, and more seeds of the same kind would not change that.
  - **The collision difference IS separable.** The execution-aware filter has 0 % envelope
    collisions in all five seeds against the margin-0 QP's 0-16 % (mean 7.2 %); the hierarchical
    interval is [-13.2, -2.4] points and EXCLUDES zero. The safety claim - the look-ahead
    constraint buys the margin-0 completion without the margin-0 collisions - is the part of the
    result that survives the extra seeds, and it is now supported by an interval rather than by
    an eyeball.
  - The episode cache is now PER CELL, so a cell that has already been evaluated is not
    re-evaluated when the seed list grows (the two new seeds cost only their own cells).
  - The claim checker grew to **198 claims** (from 156), including the five-seed intervals and
    the per-seed table of the new subsection 7.7.1.
- **The confirmatory table REPLICATES on a second untouched range.** Manifest v2's replication
  range 30000-30199 was run with the same policies, the same margins and the same three
  detectors. Every qualitative claim of section 7.7 holds, cell for cell, in all three training
  seeds: the unfiltered policy collides in about a third of the episodes under the envelope
  detector (32.0 %, 33.3 % corrected) while the mesh checker finds **no real contact in any
  cell**; the bounded QP (4 cm) and the execution-aware filter are collision-free in every cell;
  the margin-0 QP reaches the completion of the execution-aware filter (56.0 % against 56.7 %)
  and pays 7.3 % envelope / 16.0 % corrected contacts for it, and the execution-aware filter pays
  none. The absolute numbers are slightly higher on the new range (the scenarios are easier),
  but the ordering and the collision verdict are identical. The manifest version differs (the
  replication range only exists in v2), but the TEST range is identical in v1 and v2, so the
  confirmatory result is unaffected by the version change.
- **Figure 7 (the stopping tail)** is drawn from the committed JSON
  (`evaluation/plot_stopping_tail.py` -> `assets/learning/stopping_tail.png`): the tool-point
  travel after the command is cut, and the clearance lost during the tail, both against the
  filter's 100 ms horizon and its 10 mm margin. The figure also shows the fact that the median
  state does NOT lose clearance - most sampled states are far from the wall and the coast moves
  the arm away - so the erosion is in the third of the states that are close to it. That is in
  the caption, because a figure that only showed the worst case would be misleading.

## 2026-11-09: the main line is revised, and three claims are corrected
- **The main line changed** (see `docs/RESEARCH_PLAN.md` section 0, and the README's new M6
  section). The control line is demoted from "the contribution" to "the substrate": it produced
  the only *measured* results so far, and it provides exactly what a high-level agent needs - a
  **calibrated risk model of its own execution layer**. The new contribution is one level up:
  *when the safety layer's guarantee is known to be imperfect, and its imperfection is measured,
  how should an agent decide when to keep going, slow down, retreat or give up?* The decisive
  experiment compares a hand-written state machine (the strong baseline), a prompt-only agent,
  SFT and GRPO/RLVR on unseen fault combinations.
- **Three claims were corrected, and all three came from an external review of the code.** Each
  was checked against the source before anything was changed:
  1. **The execution-gap metric was measuring the wrong thing.** It counted every step that ended
     below the margin, including the steps that **started** below it - and the rate-based
     constraint *demands* escape when the arm is inside the margin, so 20 ms is often not enough.
     The metric is now split into `initially unsafe` and `safe-to-unsafe crossing`. The corrected
     numbers: at a 2 cm margin the kinematic QP leaves the arm **inside the margin buffer in
     9.1 % of its steps** (32.1 % at 4 cm) while the execution-aware filter does so in **0.2 %**
     (1.9 %) - a factor of 50 - and the safe-to-unsafe crossings are 0.2 % against 0.5 %. The old
     "9 % of the steps" claim was not defensible as a causal statement, and the effect it was
     pointing at is **larger** than stated, not smaller.
  2. **The solver latency statistic was an overwrite, not a median** (`stats["solve_time_s_median"]
     = info["time_s"]` on every step). The samples are now collected and the median, P95 and P99
     are computed from them.
  3. **The mesh checker ran on every fifth episode**, so the envelope rate and the mesh rate in
     the same table did not share a denominator. The re-run uses `--independent-stride 1`.
- **The censoring diagnostic is the important new experiment.** The episode is TERMINATED when the
  conservative envelope touches the obstacle - so the arm is stopped before it can touch for
  real, and "the mesh checker found no contact" was a statement about a truncated trajectory.
  `evaluation/physical_contact.py` runs the same policies with `terminate_on_contact=False` and
  measures the physical outcome. **The first measurement already changes a headline: the
  unfiltered policy's mesh-contact rate is 0 % censored and 50 % uncensored.** The filtered
  policies never enter that regime at all (their trajectories are identical in both modes). The
  honest statement is therefore not "the arm never really collided" but "the arm was stopped
  before it could - and the filters are the ones that keep it out of that regime".
- **The strong baseline the review asked for is in**: `speed_dependent_margin` (B4+) applies the
  textbook stopping-distance margin `m(v) = m0 + v_close * tau + v_close^2 / (2 a_min)` to the
  CLOSING speed, so the arm moving fast away from the wall needs no extra margin. If that trivial
  rule matches the look-ahead filter, the expensive method is not earning its place.
- **Nine training seeds** are now available (0-8, the archived configuration). The statistical
  verdict is stable and, for the collision difference, stronger: execution-aware - QP (0 cm) is
  **-7.3 points** of envelope collisions with a hierarchical interval of **[-11.1, -4.0]** that
  excludes zero, on nine clusters. The completion difference over the safe fixed-margin QP stays
  **not established** (+6.7 points against a 7.1-point spread) - and the reason is now the
  substantive one, not the seed count.
- The claim checker now verifies **260** claims, all matching.
- **The censoring diagnostic is DONE, and it corrects this project's own headline.** The episode
  is terminated at the first conservative-envelope contact, so the trajectory the mesh checker
  sees is truncated and the arm never gets the chance to touch for real. Measured directly (same
  policies, same episodes, the termination removed; `evaluation/physical_contact.py`):
  | method | envelope contact | mesh contact, CENSORED | mesh contact, UNCENSORED |
  |---|---|---|---|
  | unfiltered policy | 39.3 % | **0.0 %** | **22.0 %** |
  | bounded all-pair QP (4 cm) | 0.0 % | 0.0 % | 0.0 % |
  | execution-aware (1 cm) | 0.0 % | 0.0 % | 0.0 % |
  Per seed the uncensored mesh contact is 22 % / 28 % / 16 %. **So the archived reading - "the
  arm never touched the wall, only its conservative envelope" - was wrong: the arm was stopped
  before it could, and in 16-28 % of those episodes it would have.** And the FILTERS' result is
  stronger than the archived table showed: neither filtered arm ever reaches the envelope, so
  their censored and uncensored trajectories are identical and their mesh contact is 0 % in both.
  A reader who had taken the archived table at face value would have concluded the filters bought
  nothing; the measurement shows they prevent contact.
  This is now stated in the manuscript's section 4.5 (which no longer claims "not one of the 300
  episodes contains a real mesh collision"), in the new section 7.11, in the abstract, in
  docs/RESULTS.md section 12 and in the README.
- **The B4+ baseline was a straw man on the first attempt, and the fix is the interesting part.**
  The speed-dependent margin `m(v) = m0 + v_close*tau + v_close^2/(2*a_min)` with `a_min = 1`
  m/s^2 reaches HALF A METRE for a fast approach, the constraint becomes infeasible everywhere,
  and the filter becomes actively harmful - it then drives the arm into the obstacle (0 %
  completion, MORE collisions than no filter at all). The fair version CAPS the margin at the same
  4 cm the tuned fixed-margin QP uses, so B4+ can only be safer than B4, never more
  conservative: it spends the margin where the risk is (1.5 cm approaching slowly, 4 cm
  approaching fast). The parameters are now `a_min = 2.0` m/s^2 (a plausible Cartesian
  deceleration) and `tau = 22 ms` (the measured velocity-tracking lag of the servo).
- **B4+ is DONE, and the latency objection is WITHDRAWN.** The two objections that could have
  killed the result were tested:
  1. **The cheap strong alternative.** A speed-dependent stopping-distance margin
     `m(v) = min(4 cm, 1 cm + v_close*22 ms + v_close^2/(2*2 m/s^2))`, applied to the CLOSING
     speed, capped at the same 4 cm the tuned fixed-margin QP uses (so it can only be safer than
     it, never more conservative). **It does not reproduce the result**: 46.0 % completion against
     the fixed QP's 45.3 % - it does not reach the margin-0 completion of 54.0 % - and it pays
     **8.7 % corrected collisions** (per seed: 4 %, 6 %, 16 %) where the fixed 4 cm margin has
     none and the look-ahead filter has none. The adaptive margin is LESS conservative on average
     (1.5 cm instead of 4 cm when approaching slowly), and that is exactly where the contacts come
     from. The look-ahead constraint knows where the arm WILL be; the stopping-distance margin only
     knows how fast it is closing.
     (The first attempt at this baseline was a straw man: with a_min = 1 m/s^2 the margin reaches
     half a metre, the constraint is infeasible everywhere, and the filter drives the arm INTO the
     obstacle - 0 % completion and more collisions than no filter. The cap is what makes it fair.)
  2. **The latency.** The manuscript claimed the execution-aware filter costs ~8x more per control
     step. That came from the defective statistic (the "median" was the LAST solve, overwritten
     every step). With the statistic fixed: **0.890 ms median against the QP's 0.755 ms - 1.18x,
     and 1.05x at the P95 - 4.5 % of the 20 ms control period.** The objection does not survive
     its own investigation.
  Both are now in the manuscript (section 7.12), in docs/RESULTS.md (section 13), in the README
  (M5.9) and in the falsification criterion of docs/RESEARCH_PLAN.md, which is re-examined
  rather than rewritten. **Falsification criterion 1: the collision-rate part is now the
  STRONGEST part of the result (the cheap alternative does not match it), the latency part is
  withdrawn, and the completion part remains not established** - with the substantive reason (the
  between-seed spread is of the same size as the difference), not the seed count.
- **The latency is now collected properly**: the samples are kept, the median/P95/P99 are computed
  from them, and the report stores the summary plus a fixed histogram rather than fourteen
  thousand raw floats (the report went from 4.6 MB to 0.03 MB and the distribution is preserved).
- **Figure 6 was redrawn** with the corrected metric: the left panel now shows the steps the arm
  spends INSIDE the margin (the quantity that separates the two formulations) and the crossing
  rates are annotated, because they are two orders of magnitude smaller and would be invisible on
  the same axis. The README caption was rewritten to match.
- **The agent line's task had three real defects, all found by running it**: the inspection hold
  cost five tool calls (it is now ONE call that holds the arm still), the waypoint follower was
  rebuilt on every execution (so the arm walked back to the start of the path), the episode was
  6 s long (a path around the wall needs 30 s), and the FSM did not notice that the target had
  changed. The inspection points are now derived from the episode's own goal configuration by a
  base-joint sweep, so they are reachable by construction - the previous fixed Cartesian offsets
  put them in a region the planner connects to in only two thirds of the attempts, which made the
  episodes a measure of the planner's luck rather than of the agent's decisions.
