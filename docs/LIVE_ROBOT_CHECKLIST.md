# LIVE_ROBOT_CHECKLIST — 现场只读同步核验

> 适用：CR10A + EMBODIED-X LIVE_OBSERVE（只读）。全程不发送任何控制命令。
> 每一项都记录：日期、操作人、结果（✅/❌/备注），以及截图或日志路径。结果提交到 `docs/HARDWARE_COMPATIBILITY.md`。
> 任何一项出现异常，就停止后续步骤。

## 0. 前置（不接触机器人）
- [ ] 已获得所属单位对"连接该机器人控制柜网络（只读）"的授权；了解现场急停位置和安全规定
- [ ] 笔记本已通过：`bash scripts/test.sh`（单元测试全部通过）和 `bash scripts/integration_test.sh`（ALL STAGES OK）
- [ ] 确认 `ex_bringup/config/modes/live_observe.yaml` 当前是 `enable_live_connection: false`（现场再改）
- [ ] **不要**启动官方 `dobot_bringup_v4`，它会连接 29999

## 1. 身份与版本（只看，不连接）
- [ ] 铭牌：型号 ______（是否写 CR10A）、序列号 ______
- [ ] 示教器 / DobotStudio Pro 中的控制器版本：______（协议文档对应 V4.x；若是 V3.x，**停止**，改用 V3 协议）
- [ ] 控制柜 IP：______（默认有线 192.168.5.1）；笔记本网卡设为同网段的静态 IP
- [ ] 当前工具坐标系编号 ______、用户坐标系编号 ______（交叉校验要求 tool 0 / user 0；不要为此去修改设置，记录下来即可）
- [ ] 当前负载设置 ______ kg（只记录，不修改）

## 2. 网络层只读验证（机器人保持在当前状态，最好处于下使能 / 静止）
1. WSL2 与控制柜的连通性：WSL 默认是 NAT 模式，出站 TCP 一般能通。
   官方 README 只列出 29999（控制）和 30004（反馈）；协议 PDF V4.6.5 §1 另写有 30005/30006。
   **你这台控制器实际开了哪些端口未知**，所以先逐个只读探测反馈端口（**不要探测 29999**）：
   ```bash
   source scripts/ex_env.sh
   for p in 30004 30005 30006; do
     n=$(timeout 3 bash -c "cat < /dev/tcp/192.168.5.1/$p | head -c 1440 | wc -c" 2>/dev/null)
     echo "port $p -> ${n:-closed} bytes"      # 1440 表示端口开放并推送数据；只读，不写入
   done
   ```
   - [ ] 30004 = ______；30005 = ______；30006 = ______
2. 第一次连接：如果 30005 开放，先用 **30005（200 ms，低频）**；否则直接用 **30004**（官方驱动用的就是这个端口）：
   - [ ] 把 `live_observe.yaml` 改为 `enable_live_connection: true`，并设 `adapter.expected_rate_hz: 5.0`
   - [ ] `ros2 launch ex_bringup embodied_x.launch.py mode:=live_observe robot_ip:=192.168.5.1 feedback_port:=30005 live_ack:=I_AM_ON_SITE_AND_AUTHORIZED`
   - [ ] 观察 `/diagnostics` 中 `ex/dobot_feedback_observer`：`state=STREAMING`，`resyncs=0`，`decode_errors=0`
   - [ ] **半关闭测试**：如果日志反复出现 `peer closed connection`，说明控制器不接受 FIN → 改为 `half_close_write: false` 后重试，并在此处记录
   - [ ] DobotStudio Pro 是否仍然正常（连接状态、能否正常操作）：______
3. 切换到 **30004（8 ms）**：
   - [ ] HUD 显示约 125 Hz、FRESH；`clock_offset_ms_median` / `stdev` = ______ / ______

## 3. 关节映射与单位（机器人静止）
- [ ] 示教器 J1..J6 读数（度）：______
- [ ] HUD 上 joint1..joint6（deg）：______ → 逐一相等（允许 ±0.01°）。检查顺序、符号和单位
- [ ] 通过合规方式（示教器点动，或在授权下拖拽）**单独**移动 J1，只有 RViz 中的 J1 动，方向一致；J2..J6 依次重复同样的检查
- [ ] RViz 姿态和真机目视一致（拍摄并排照片 → `assets/`，注意单位的公开规定）

## 4. 运动学模型判定（核心实验，约 2 分钟）
1. 确认 tool 0 / user 0（看 HUD 诊断里的 `tool_coordinate` 和 `user_coordinate`）
2. 录制：`bash scripts/record_bag.sh cr10a_live_01 120`，同时让操作员以合规方式让机器人做**多关节的大范围运动**（越多样越好）
3. 离线分析（可以回到办公室再做）：
   ```bash
   python3 evaluation/crosscheck_bag.py data/bags/cr10a_live_01 --out data/reports/cr10a_model_id.json
   ```
   - [ ] `best_position_match` = ______；rms = ______ mm；max = ______ mm
   - [ ] 判读：rms 只有几 mm 或更小，说明模型正确（残差来自理论 DH 与标定 DH 的差异）；约 42 mm，说明法兰定义选错（换 cr10af）；几十到上百 mm 且随姿态变化，说明几何不符或映射/单位有问题
   - [ ] `best_orientation_hypothesis` = ______（确定 rx/ry/rz 的欧拉约定和四元数顺序）
4. 根据结果更新 `HARDWARE_COMPATIBILITY.md` 的第 2、6、9 行，并用 `scripts/vendor_dobot_cr10.py` 生成最终模型

## 5. 异常注入（可选，只读）
- [ ] 拔掉网线 3 s 再插回：HUD 依次显示 STALE → LOST、INVALID、显示冻结；恢复后出现 RESYNC 警告，没有跳变；`disconnects` +1
- [ ] 结束：Ctrl-C；把 `enable_live_connection` 改回 `false`；在 PROGRESS.md 写入现场记录

## 6. 激励轨迹录制（动力学辨识，**会让机器人运动**）

只有完成 §0–§3、并获得所属单位允许程序控制的授权后才能进行。

- [ ] 现场有人看护，**手放在急停旁**；机械臂周围半径 1.5 m 内没有人和障碍物；末端没有装工具，或者实测了工具质量 ______ kg
- [ ] 示教器上碰撞检测等级 ______（不得为 0）；控制器没有报警
- [ ] 用示教器或 DobotStudio 把机械臂放到**开阔姿态**（手臂前伸、腕部朝下、离桌面 ≥ 0.4 m），然后**下使能**，并关闭 DobotStudio 的控制连接
- [ ] 演练：`python3 -m ex_dobot_motion.runner --robot-ip 192.168.201.1` → 必须显示 `validation: OK`，看一下 TCP 范围
- [ ] 执行：加上 `--execute --confirm I_AM_AT_THE_ROBOT_WITH_ESTOP --payload-kg <工具质量>`（速度默认 20%，大约 2–4 分钟）
- [ ] 观察：运动平滑、幅度小；有任何异常就按急停，或在终端按 Ctrl-C
- [ ] 结束：机器人回到起点并下使能；数据在 `data/live/excitation_*`（feedback.csv.gz 包含 q、qd、电流、力矩）
