# CLAUDE.md — EMBODIED-X working rules

## Context
Long-running embodied-AI research platform around a **Dobot CR10A** (+ Jetson TX2, RealSense T265).
Owner: AI algorithm engineer (ROS 1 / SLAM / LLM background). Public portfolio project → honesty and
reproducibility matter more than polish. Mixed Chinese/English explanations are welcome.

## Hard safety rules
1. **Never connect to a real robot / non-loopback IP** without explicit on-site authorization from the
   owner in the current conversation. Never flip `enable_live_connection` to true in committed config.
2. Milestone 1 is **read-only**: no code may open Dobot port 29999 (or 30003) or send bytes to a
   controller. Keep `test_readonly_socket.py` passing; never weaken the port allowlist.
3. Do not launch official `dobot_bringup_v4` (opens 29999, exposes ~130 unauthenticated services).
4. Motion-capable code, when it exists, is disabled by default and needs separate review.
5. Never fabricate results. Anything not run is "UNVERIFIED". Mock/simulated data is labelled as such.

## Environment
- Develop in WSL distro **`EmbodiedX-Humble`** (Ubuntu 22.04, ROS 2 Humble, user `ex`).
  **Do not touch `Ubuntu-22.04` or `Ubuntu-24.04`** (other projects of the owner).
- Repo lives at `C:\Users\HGDOCTOR\embodied-x` (= `/mnt/c/Users/HGDOCTOR/embodied-x`); build output in
  `~/ex_ws_out` (Linux FS). `source scripts/ex_env.sh` sets ROS env, `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`.
- WSL NAT cannot reach the Windows proxy (Clash TUN); downloads from Windows side work. GPU: RTX 5060 Laptop 8 GB.
- Files must be LF (`.gitattributes`). Editing via Windows Python `write_text` produces CRLF — avoid / convert.

## Commands
`bash scripts/build.sh` · `bash scripts/test.sh` · `bash scripts/integration_test.sh` ·
`bash scripts/run_demo.sh <mode>` · `python3 scripts/vendor_dobot_cr10.py` (regenerate description)

## Git / backup
- Remote: `origin` = https://github.com/terrense/experiment_lab_embodied_intelligence_CR10A (**public**).
- After every verified step (tests run, docs updated): commit and `git push origin main`. Never leave
  finished work only on the local disk. A repo-local proxy (`http.https://github.com/.proxy`) is set.
- Public repo: no credentials, no details of the owner's other projects. The owner approved publishing
  real-robot data (2026-10-27); still curate it: raw bags stay out of git (size), curated evidence goes to
  `docs/evidence/`, screenshots are cropped to the app window.

## Engineering conventions
- ROS-free logic in `ex_robot_state/core/` with unit tests; nodes are thin wrappers.
- Units: radians in ROS messages; Dobot packet `QActual` is degrees. Frames: world → base_link → … → Link6 → flange → tcp.
- Third-party code: pinned in `third_party/MANIFEST.yaml`, fetched not committed; vendored pieces via scripts with provenance headers.
- Every milestone updates `docs/PROGRESS.md` with commands actually run, test results, blockers, next steps.
- Avoid premature abstraction: generalize only after a second real use case.
